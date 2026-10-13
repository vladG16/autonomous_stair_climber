#!/usr/bin/env python3
"""
Live YOLO + ByteTrack view from the Jetson, streamed to any browser on the network.

Run on the Jetson:   python3 live_view.py --source 0
Open on the laptop:  http://<jetson-ip>:8080
Stop with Ctrl+C.
"""
import argparse
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from ultralytics import YOLO

parser = argparse.ArgumentParser()
parser.add_argument('--source', default='0', help='camera index (0) or device path (/dev/video0)')
parser.add_argument('--model', default=os.path.expanduser('~/ros2_ws/models/yolov8n.engine'))
parser.add_argument('--imgsz', type=int, default=416)
parser.add_argument('--port', type=int, default=8080)
parser.add_argument('--conf', type=float, default=0.5)
args = parser.parse_args()

latest_jpeg = None
lock = threading.Lock()
running = True


def capture_loop():
    global latest_jpeg
    model = YOLO(args.model, task='detect')
    dev = f'/dev/video{args.source}' if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)   # open by path: more reliable than by index
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPG fits USB 2.0 bandwidth
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print(f'Could not open {dev}. Check it is not in use: sudo fuser {dev}')
        os._exit(1)
    print(f'Opened {dev}: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
          f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

    t_prev = time.time()
    fps = 0.0
    while running:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        t0 = time.time()
        res = model.track(frame, persist=True, classes=[0], conf=args.conf, imgsz=args.imgsz,
                          tracker='bytetrack.yaml', verbose=False)[0]
        infer_ms = (time.time() - t0) * 1000
        annotated = res.plot()

        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-6))
        t_prev = now
        cv2.putText(annotated, f'{fps:4.1f} FPS | track+infer {infer_ms:4.1f} ms',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        ok, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest_jpeg = buf.tobytes()
    cap.release()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            page = (b'<html><body style="margin:0;background:#111">'
                    b'<img src="/stream" style="width:100%;max-width:960px;display:block;margin:auto">'
                    b'</body></html>')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(page)
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while running:
                    with lock:
                        jpg = latest_jpeg
                    if jpg is not None:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                    time.sleep(1 / 30)
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser tab closed
        else:
            self.send_error(404)

    def log_message(self, *a):
        pass  # keep the terminal quiet


if __name__ == '__main__':
    threading.Thread(target=capture_loop, daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    print(f'Streaming on http://<jetson-ip>:{args.port}  (Ctrl+C to stop)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        running = False
        server.shutdown()