# RB3 Codebase

This folder contains the code that runs on the RB3 board. It includes:

### 1. Image Capture
Captures real-time video or images using a connected camera.

### 2. Vision Pipeline
Uses a YOLO-based object detection pipeline to detect and classify objects in the scene.

### 3. Logic Node
This ROS2 node performs decision-making based on vision output and sends appropriate control messages to the Jetson Nano.

## Structure
rb3/
├── image_capture/ # Camera interface and image acquisition
├── yolo_pipeline/ # YOLO model code and inference logic
├── logic_node/ # ROS2 node that performs decision making
└── README.md # This file

## 🛠 Setup & Run Instructions

```bash
export ROS_DOMAIN_ID=42
cd /home/ubuntu/bin/ros2_ws
source /opt/ros/humble/setup.bash 
colcon build 
source install/setup.bash
```

Then in a tmux session

```bash
# Terminal 1
sh run_camera_node.sh
# Terminal 2
sh run_detection_node.sh
# Terminal 3
sh run_logic_comm.sh
```


## Dependencies
- ROS2 Humble
- OpenCV
- YOLOv4
- Python 3.10.12