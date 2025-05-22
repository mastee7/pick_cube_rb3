# RB3 Codebase

This folder contains the code that runs on the RB3 board. It includes:

### 1. Image Capture
Captures real-time video or images using a connected camera.

### 2. Vision Pipeline
Uses a YOLO-based object detection pipeline to detect and classify objects in the scene.

### 3. Logic Node
This ROS2 node performs decision-making based on vision output and sends appropriate control messages to the Jetson Nano.

---

## 📁 Folder Structure
```
.
├── rb3/ # Perception and planning
├── README.md
└──run_camera_node.sh
    ├── run_detection_node.sh
    ├── run_logic_comm.sh
    └── src
        ├── camera_node
        │   ├── camera_node
        │   │   └── image_publisher_node.py
        ├── detection_node
        │   ├── detection_node
        │   │   └── detection_subscriber_node.py
        │   ├── resource
        │   │   └── detection_node
        │   ├── setup.cfg
        │   ├── setup.py
        │   └── share
        │       └── resource
        │           ├── data
        │           │   ├── obj.names
        │           │   └── yolov4-tiny.cfg
        │           └── yolov4-tiny_last.weights
        └── rb3_nano_comm
            ├── rb3_nano_comm
            │   └── rb3_publisher.py
            ├── resource
            │   └── rb3_nano_comm
            ├── setup.cfg
            └── setup.py
```

## 🛠 Setup & Run Instructions

```bash
source /opt/ros/humble/setup.bash 
colcon build --packages-select  camera_node
source install/setup.bash
```

Then, in a tmux session:

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
- YOLOv4
- Python 3.10.12