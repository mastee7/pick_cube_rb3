# Jetson Nano Codebase

This folder contains the control-related code running on Jetson Nano. It is responsible for:

- Receiving commands from the RB3's logic node
- Sending appropriate signals to actuators (e.g., motors)

## Structure
```
jetson_nano/
└── README.md
    └── src
        └── rb3_nano_comm
            ├── rb3_nano_comm
            │   └── nano_subscriber.py
            ├── resource
            │   └── rb3_nano_comm
            ├── setup.cfg
            └── setup.py
```

## Features
- Serial communication or direct GPIO motor control
- Real-time command handling and execution
- Supports different motor drivers (specify in configs)

## 🛠 Setup & Run Instructions

```bash
export ROS_DOMAIN_ID=42
cd /home/jetson/recover/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build # --packages-select rb3_nano_comm
source install/setup.bash
ros2 run rb3_nano_comm nano_sub

## Dependencies
- ROS2 Foxy

