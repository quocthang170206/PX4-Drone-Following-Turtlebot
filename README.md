# 🛸 PX4 Drone Following TurtleBot Using YOLOv8 (ROS 2)

This project demonstrates a PX4 drone running in Gazebo that autonomously follows a randomly moving TurtleBot3 using a downward-facing camera and real-time object detection powered by YOLOv8.

---

## 📁 Repository Structure
```text
ros2_ws/
└── src/
    ├── drone_takeoff_pkg/           # Drone takeoff and OFFBOARD control
    ├── turtlebot_random_move_pkg/   # FSM-based TurtleBot random movement
    └── yolo_detection_pkg/          # YOLOv8 detection + tracking logic
        └── weights.pt               # Trained YOLOv8 model
```
---

## 🧠 Features

- PX4 SITL drone simulation
- Real-time YOLOv8 object detection using `ultralytics`
- MAVROS velocity control in OFFBOARD mode
- Random TurtleBot movement with obstacle avoidance
- Drone maintains position directly above TurtleBot

---

## ✅ Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- PX4-Autopilot (`px4_sitl`)
- MAVROS
- Gazebo Classic
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- Python 3.10+

---

## 🛠️ Build Instructions

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
1. Launch PX4 SITL with Gazebo

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    cd ~/PX4-Autopilot
    make px4_sitl gazebo
    ```

2. Start MAVROS to connect with PX4

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@
    ```

3. Spawn TurtleBot in Gazebo

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run gazebo_ros spawn_entity.py \
      -entity turtlebot3 \
      -file ~/turtlebot3_models/turtlebot3_burger/model.sdf \
      -x 3 -y 3 -z 0.0
    ```

4. Takeoff and switch to OFFBOARD mode

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch drone_takeoff_pkg takeoff_launch.py
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
    ```

5. Launch YOLOv8-based visual tracking

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch yolo_detection_pkg yolo_detection_launch.py
    ```

6. Start TurtleBot random movement

    ```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run turtlebot_random_move_pkg random_move
    ```

> 💡 **Note**: The TurtleBot operates in a restricted area and avoids obstacles using 2D LaserScan.
