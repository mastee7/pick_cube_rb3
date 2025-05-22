# RB3 + Jetson Nano Robotics Project

This repository is organized into two main folders:

- **`rb3/`**: Contains code running on the **Qualcomm RB3 board**, including image capture, object/pose detection using YOLO, and the logic node that manages perception and planning.
- **`jetson_nano/`**: Contains code running on the **Jetson Nano**, specifically for motor control and executing commands received from the RB3.

Each folder has its own `README.md` for more details.

## Folder Structure
```
.
├── rb3/ # Perception and planning
└── jetson_nano/ # Control
```