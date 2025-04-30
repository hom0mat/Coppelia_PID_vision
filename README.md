# 🎱 Ball Follower Robot – CoppeliaSim + ROS2 + Python

## 🤖 Project Description

This project simulates a robot that follows a moving sphere in **CoppeliaSim**, with real-time control via **ROS2** and movement generated using **Python** scripting. You'll find the ROS2 package for this project in act1_2 folder, and the required **CoppeliaSIm** scene in coppelia_scene folder.

## 🚀 Features

- Simulates smooth sinusoidal or circular motion of a sphere.
- Robot vision integration for position detection.
- Feedback-based movement to follow the ball in real-time.
- Modular ROS2 node architecture.
- Easy to customize motion parameters (amplitude, frequency, etc.)

## 🛠️ Technologies Used

- [CoppeliaSim](https://www.coppeliarobotics.com/)
- Python 3
- ROS2 (tested on Humble/Foxy)
- ZeroMQ Remote API (ZMQ)
- OpenCV (for image processing)

## ⚙️ Installation
1. Clone the repository:
git clone https://github.com/hom0mat/Coppelia_PID_vision.git 

2. Make sure zmqRemoteApi client is available:
It's usually found in the CoppeliaSim directory under programming/zmqRemoteApi/clients/python/

3. Run the ROS2 launch file:
ros2 run act1_3 launch.py

4. Make sure CoppeliaSim is open and simulation is running in stepping mode.

🌀 Switching to Circular Motion
To switch from sinusoidal to circular movement, update the move_ball() function in ball.py as follows:

x = center_x + radius * math.cos(2 * math.pi * frequency * t)
y = center_y + radius * math.sin(2 * math.pi * frequency * t)

# 🎯 Upgrades: Vision-Based Mobile Robot – FSM + CoppeliaSim + ROS2 + Python
[29 de abril de 2025]

## 🤖 Project Description

The robot receives visual data from a **vision sensor** in the simulation. Based on the detected position of the sphere in the image frame, a **finite state machine** selects the appropriate movement strategy.

This architecture ensures **robust, modular, and reactive behavior**, ideal for real-time tracking applications. You'll find the ROS2 package in the act1_4 folder, and the **CoppeliaSim** scene in act1_4/simulator_coppelia path.

## 🚀 New Features

- Real-time **vision-based tracking** of a sphere using image coordinates.
- FSM implementation for **discrete robot behaviors** based on visual input.

## 🧠 FSM States Overview

The robot uses a Finite State Machine (FSM) to determine its behavior based on the color of the detected sphere from the vision sensor. Each color corresponds to a specific movement directive:

🟢 Green = FORWARD	Move forward toward the sphere using PID control for alignment.
🟠 Orange	TURN_RIGHT	Turn right until a green sphere is detected again.
🔵 Blue	TURN_LEFT	Turn left until a green sphere is detected again.
🟡 Yellow	PAUSE	Pause for 5 seconds before resuming operation.
⬛ None	STOP	No sphere detected – stop all movement.

The FSM transitions between these states in real time based on visual feedback, allowing the robot to reactively plan its local path. PID control is integrated in the FORWARD state to smoothly align the robot with the sphere's position.

This combination of FSM logic and PID control enables a robust and flexible behavior strategy for autonomous navigation in dynamic environments.

Each transition is driven by the horizontal position of the target within the image from the vision sensor.

## 🛠️ Technologies Used

- [CoppeliaSim](https://www.coppeliarobotics.com/)
- Python 3
- ROS2 (tested on Humble/Foxy)
- ZeroMQ Remote API (ZMQ)
- OpenCV (for image processing, if used)

👩‍💻 Author: César Mateo Sánchez Álvarez

Robotics & Embedded Systems Student 💻🤖

A01541805@tec.mx | Tecnológico de Monterrey
