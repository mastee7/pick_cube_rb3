# RB3 Color Cube Perception and Grasping


<a href='https://mastee7.github.io/'><strong>Woojeh Chung</strong></a>* · 
<a href='https://hjhyunjinkim.github.io/'><strong>Hyunjin Kim</strong></a>* · 
<a href='https://dlwjddms.github.io/'><strong>Jeongeun Lee</strong></a>*

(* Equal Contribution)

Supervised by <a href='https://hichristensen.com'><strong>Henrik I. Christensen</strong></a>, <a href='https://www.linkedin.com/in/skandal/'><strong>Sriraj Kandala</strong></a>, <a href='https://www.linkedin.com/in/ramakrishna-kintada-6903564/'><strong>Ramakrishna Kintada</strong></a>

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
