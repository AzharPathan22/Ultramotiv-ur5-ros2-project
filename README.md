# Ultramotiv Robotics Developer Assignment – UR5 6-DOF Arm

This project demonstrates a complete robotic arm system using **ROS 2 Humble**, **Gazebo**, **MoveIt 2**, and **YOLOv8**. The system is built around a **UR5 6-DOF robotic arm** and includes simulation, motion planning, kinematics-based control, and vision-based object detection.

---

## Features

- 6-DOF UR5 robotic arm simulation
- MoveIt 2 integration for trajectory planning
- Custom inverse and forward kinematics implementation
- Joint angle control via ROS 2 publishers
- Simulated RGB-D camera mounted on robot
- YOLOv8 object detection on camera feed

---

## Demo Videos

| Task       | Description                                      | Demo                                                                                                |
| ---------- | ------------------------------------------------ | --------------------------------------------------------------------------------------------------- |
| **Task 1** | UR5 Simulation + MoveIt 2                        | [Watch Video](https://drive.google.com/drive/folders/1xwrO1HW_mbubSeiclnIKP5XYTQKB_fHX?usp=sharing) |
| **Task 2** | Kinematics Node (IK/FK) + Joint Control          | [Watch Video](https://drive.google.com/drive/folders/1xycqfqOt0hPft47nLs3ByNjxfzmWaL4W?usp=sharing) |
| **Task 3** | Object Detection using Simulated Camera + YOLOv8 | [Watch Video](https://drive.google.com/drive/folders/1y4LrgmBsd4sU1jxTrJPT2myHaOVmtyBN?usp=sharing) |

---

##  Requirements / Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (Fortress or compatible)
- Python 3.10+
- [`ultralytics`](https://github.com/ultralytics/ultralytics) (YOLOv8)
- OpenCV
- `cv_bridge`, `sensor_msgs`, `trajectory_msgs`
- [`IKPy`](https://github.com/Phylliade/ikpy)

## Install Python dependencies:

```bash
pip install -r requirements.txt
```

---

## Installation Instructions

```bash
git clone --recurse-submodules <repo-url> ultramotiv_ws
cd ultramotiv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

# TASK 1 – SIMULATION & MOVEIT 2

### Description

Simulates the UR5 robotic arm in **Gazebo** and visualizes it in **RViz**. Integrates **MoveIt 2** to plan and execute trajectories.

### Run Instructions

```bash
ros2 launch ur5_sim spawn_ur5_moveit.launch.py
```

### Topics Used

- `/joint_states` (Published)
- `/tf` (Published)
- `/move_group` action interface (MoveIt)

### Notes

- You can use the interactive marker in RViz to set a new pose and click “Plan” and then “Execute”.
- TF and JointStatePublisher work in sync.

---

# TASK 2 – KINEMATICS CONTROL (FK/IK)

### Description

Implements forward and inverse kinematics using **IKPy**. Accepts 3D target position as input, computes joint angles, and publishes them to move the robot in Gazebo.

### Run Instructions

```bash
# Compute FK and IK
ros2 run ur5_kinematics ik_command_node --ros-args -p x:=0.5 -p y:=0.4 -p z:=0.8
```

### Topics Used

- `/joint_trajectory_controller/joint_trajectory` (Published)
- `/joint_states` (Subscribed)

### Notes

- The IK node loads the robot’s URDF dynamically.
- Uses 6 active links including wrist and base.
- You can send different XYZ goals for testing.

---

#  TASK 3 – OBJECT DETECTION WITH YOLOv8

###  Description

A **simulated RGB camera** is mounted on the robot’s end-effector in Gazebo. Captures live feed, runs **YOLOv8 detection**, and shows real-time bounding boxes using OpenCV.

###  Run Instructions

```bash
ros2 run ur5_yolov8_detector yolo_node
```

### Topics Used

- `/camera/image_raw` (Subscribed)
- `cv2.imshow()` (Pop-up detection window)
- Terminal logs for object classes and confidence scores

### Notes

- Uses `ultralytics` YOLOv8 model (e.g., yolov8n.pt)
- Simulated camera is placed to view the objects on the table.
- You may add custom textures or PNG images to improve detection.

---

## Launch File

| File                         | Description                                       |
| ---------------------------- | ------------------------------------------------- |
| `spawn_ur5_moveit.launch.py` | Spawns robot in Gazebo + RViz + Launches MoveIt 2 |

---

## Project Directory Structure (Simplified)

```
ultramotiv_ws/
├── src/
│   ├── ur5_sim/
│   ├── ur5_kinematics/
│   └── yolov8_detector/
├── install/
├── build/
├── log/
├── README.md
└── requirements.txt
```

---

## Usage Notes

- Use XYZ target values that are reachable by UR5.
- Make sure to position objects where the camera can see them.
- You can add more objects using SDF or include URDF models.

---

## Troubleshooting

- **No detection output?**
  - Ensure camera is publishing `/image_raw`
  - Add larger, common YOLOv8-detectable objects (bottle, cup)

- **CV Bridge error?**
  - Install with: `sudo apt install ros-humble-cv-bridge`

---

## Credits / Acknowledgements

- Universal Robots UR5 model
- MoveIt 2
- Gazebo Sim
- IKPy library
- YOLOv8 (Ultralytics)

---

 **Thank you for reviewing my assignment.**  
 If you'd like to test any part or need assistance, feel free to contact me.

---
## Contact

Azhar Pathan
azarpathan888@gmail.com
