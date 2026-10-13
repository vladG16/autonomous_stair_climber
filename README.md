#  Person Follower
 
ROS 2 Jazzy package for the FAU Group 5 stair-climbing delivery cart. YOLOv8 + ByteTrack detects and locks onto a person through the Orbbec Astra Pro Plus, and the node publishes `/cmd_vel` to follow them.
 
**Hardware:** Jetson Orin Nano (JetPack 7.2.1, Ubuntu 24.04), Orbbec Astra Pro Plus
 
## Connect
 
```
ssh team5@192.168.1.145
```
If the IP changed, run `hostname -I` on the Jetson.
 
## Build (only after changing setup.py or adding files)
 
Venv must be **off**.
```
deactivate 2>/dev/null
cd ~/ros2_ws
colcon build --symlink-install
```
 
## Run the person follower
 
```
cd ~/ros2_ws
source install/setup.bash
yolo_on
ros2 run cart_follower person_follower --ros-args -p model:=/home/team5/ros2_ws/models/yolov8n.engine
```
`yolo_on` activates the YOLO venv. It needs the camera driver publishing `/camera/color/image_raw` and `/camera/depth/image_raw`.
 
## Live camera test (no ROS)
 
```
yolo_on
cd ~/ros2_ws/models
python3 live_view.py --source 0
```
Then open **http://192.168.1.145:8080** on a laptop on the same network.
 
## Stop
 
**Ctrl+C**, never Ctrl+Z (Ctrl+Z leaves the camera locked).
 
## Model file
 
`yolov8n.engine` is not in the repo; it is built per device. To rebuild it on the Jetson:
```
yolo_on
cd ~/ros2_ws/models
yolo export model=yolov8n.pt format=engine half=True imgsz=416 workspace=1
```
 
