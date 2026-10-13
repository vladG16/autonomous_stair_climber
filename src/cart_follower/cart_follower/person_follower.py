#!/usr/bin/env python3
"""
Astra Pro Plus + YOLOv8/ByteTrack person follower
ROS 2 Humble, Jetson Orin Nano

Input : color + depth images from the Orbbec ROS 2 driver (depth registered to color)
Output: geometry_msgs/Twist on cmd_vel  (motor hardware is NOT handled here)

Ported from the ZED version. The ZED did detection and 3D position internally;
here YOLOv8 does detection, ByteTrack keeps one person locked, and depth comes
from the median of a torso patch in the aligned depth image.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from ultralytics import YOLO


class AstraPersonFollower(Node):
    def __init__(self):
        super().__init__('astra_person_follower')

        # ---------------- Parameters ----------------
        defaults = {
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'cmd_vel_topic': '/cmd_vel',
            'model': 'yolov8n.engine',      # TensorRT engine on Jetson; .pt / .onnx also work
            'conf': 0.5,
            'imgsz': 416,
            'target_dist': 1.0,             # m. Astra depth is unreliable below ~0.6 m
            'min_depth': 0.4,
            'max_depth': 6.0,
            'kp_lin': 0.65, 'kd_lin': 0.25,
            # Angular gains act on NORMALIZED pixel error in [-1, 1].
            # Equivalent to the ZED gains: 0.0015 * 960 = 1.44, 0.0002 * 960 = 0.19
            'kp_ang': 1.44, 'kd_ang': 0.19,
            'alpha': 0.25,                  # depth low-pass (EMA)
            'deadband': 0.06,               # m
            'max_lin': 0.45, 'max_ang': 0.5, 'max_accel': 0.8,
            'lost_timeout': 1.5,            # s before searching / relocking a new person
            'search_enabled': True,
            'search_ang_vel': 0.4,
            'watchdog_timeout': 0.5,        # s without camera frames -> stop
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda n: self.get_parameter(n).value
        self.p = {k: g(k) for k in defaults}

        # ---------------- ROS I/O ----------------
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        color_sub = message_filters.Subscriber(self, Image, self.p['color_topic'],
                                               qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, self.p['depth_topic'],
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.frame_cb)
        self.create_timer(0.1, self.watchdog_cb)

        # ---------------- Model ----------------
        self.model = YOLO(self.p['model'], task='detect')

        # ---------------- State ----------------
        now = self.get_clock().now()
        self.last_frame_time = now
        self.last_seen_time = now
        self.last_time = now
        self.locked_id = None
        self.need_reset = True
        self.filtered_depth = 0.0
        self.prev_lin_error = 0.0
        self.prev_pix_error = 0.0
        self.last_v = 0.0

        self.get_logger().info('Astra person follower running.')

    # ------------------------------------------------------------------
    def _dt(self, now):
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        return min(dt, 0.2)  # clamp so a stalled frame can't spike the D-term

    def _secs_since(self, now, t):
        return (now - t).nanoseconds / 1e9

    # ------------------------------------------------------------------
    def frame_cb(self, color_msg, depth_msg):
        now = self.get_clock().now()
        self.last_frame_time = now

        frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        h, w = frame.shape[:2]

        res = self.model.track(frame, persist=True, classes=[0], conf=self.p['conf'],
                               imgsz=self.p['imgsz'], tracker='bytetrack.yaml',
                               verbose=False)[0]
        box = self.select_target(res, now)
        if box is None:
            self.handle_lost(now)
            return

        self.last_seen_time = now
        x1, y1, x2, y2 = box
        raw_depth = self.torso_depth(depth, x1, y1, x2, y2, w, h)

        # ---- (Re)initialize on acquisition: no derivative kick, no depth ramp ----
        bbox_cx = 0.5 * (x1 + x2)
        pix_error = (w / 2.0 - bbox_cx) / (w / 2.0)   # + = person LEFT of center -> turn left (+z)
        if self.need_reset:
            self.filtered_depth = raw_depth if raw_depth is not None else self.p['target_dist']
            self.prev_lin_error = self.filtered_depth - self.p['target_dist']
            self.prev_pix_error = pix_error
            self.last_time = now
            self.need_reset = False
            return
        dt = self._dt(now)
        if dt <= 0.0:
            return

        # ---- Linear (PD on filtered depth) ----
        if raw_depth is not None:
            a = self.p['alpha']
            self.filtered_depth = a * raw_depth + (1.0 - a) * self.filtered_depth
            lin_error = self.filtered_depth - self.p['target_dist']
            lin_deriv = (lin_error - self.prev_lin_error) / dt
            v_raw = self.p['kp_lin'] * lin_error + self.p['kd_lin'] * lin_deriv
            if abs(lin_error) < self.p['deadband']:
                v_raw = 0.0
            self.prev_lin_error = lin_error
        else:
            v_raw = 0.0  # no valid depth (too close / IR dropout): don't drive forward blind

        # ---- Angular (PD on normalized pixel error) ----
        pix_deriv = (pix_error - self.prev_pix_error) / dt
        w_cmd = self.p['kp_ang'] * pix_error + self.p['kd_ang'] * pix_deriv
        self.prev_pix_error = pix_error

        # ---- Slew limit + saturation ----
        dv_max = self.p['max_accel'] * dt
        v = self.last_v + max(min(v_raw - self.last_v, dv_max), -dv_max)
        v = max(min(v, self.p['max_lin']), -self.p['max_lin'])
        w_cmd = max(min(w_cmd, self.p['max_ang']), -self.p['max_ang'])

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)
        self.last_v = v

        self.get_logger().info(
            f'ID {self.locked_id} | Dist {self.filtered_depth:.2f} m | '
            f'PixErr {pix_error:+.2f} | v {v:.2f} | w {w_cmd:+.2f}',
            throttle_duration_sec=0.5)

    # ------------------------------------------------------------------
    def select_target(self, res, now):
        """Follow the locked ByteTrack ID. Only lock a new person after lost_timeout."""
        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            return None
        xyxy = boxes.xyxy.cpu().numpy()
        ids = boxes.id.int().cpu().tolist() if boxes.id is not None else [None] * len(xyxy)

        if self.locked_id is not None and self.locked_id in ids:
            return xyxy[ids.index(self.locked_id)]

        # Locked person missing: wait out lost_timeout before switching to someone else
        if self.locked_id is not None and \
                self._secs_since(now, self.last_seen_time) < self.p['lost_timeout']:
            return None

        areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
        i = int(np.argmax(areas))           # lock the largest (usually closest) person
        self.locked_id = ids[i]
        self.need_reset = True
        self.get_logger().info(f'Locked onto person ID {self.locked_id}')
        return xyxy[i]

    def torso_depth(self, depth, x1, y1, x2, y2, w, h):
        """Median depth (m) of the torso region; avoids background between arms/legs."""
        bw, bh = x2 - x1, y2 - y1
        u0, u1 = x1 + 0.35 * bw, x2 - 0.35 * bw
        v0, v1 = y1 + 0.20 * bh, y1 + 0.50 * bh
        sx, sy = depth.shape[1] / w, depth.shape[0] / h   # handles color/depth res mismatch
        patch = depth[int(v0 * sy):int(v1 * sy), int(u0 * sx):int(u1 * sx)]
        if patch.size == 0:
            return None
        d = patch.astype(np.float32)
        if depth.dtype == np.uint16:
            d *= 0.001  # mm -> m
        d = d[(d > self.p['min_depth']) & (d < self.p['max_depth'])]
        if d.size < 20:
            return None
        return float(np.median(d))

    # ------------------------------------------------------------------
    def handle_lost(self, now):
        self.need_reset = True
        self.last_v = 0.0
        if self.p['search_enabled'] and \
                self._secs_since(now, self.last_seen_time) > self.p['lost_timeout']:
            msg = Twist()
            msg.angular.z = float(self.p['search_ang_vel'])
            self.cmd_pub.publish(msg)
        else:
            self.stop_robot()

    def watchdog_cb(self):
        if self._secs_since(self.get_clock().now(), self.last_frame_time) > self.p['watchdog_timeout']:
            self.need_reset = True
            self.last_v = 0.0
            self.stop_robot()

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AstraPersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()