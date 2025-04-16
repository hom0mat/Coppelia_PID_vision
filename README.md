# 🎱 Ball Follower Robot – CoppeliaSim + ROS2 + Python

This project simulates a robot that follows a moving sphere in **CoppeliaSim**, with real-time control via **ROS2** and movement generated using **Python** scripting.

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

## ⚙️ Installation
1. Clone the repository:
git clone https://github.com/hom0mat/Coppelia_PID_vision.git 

2. Make sure zmqRemoteApi client is available:
It's usually found in the CoppeliaSim directory under programming/zmqRemoteApi/clients/python/

3. Run the ROS2 launch file:
ros2 run act1_2 launch.py

4. Make sure CoppeliaSim is open and simulation is running in stepping mode.

🌀 Switching to Circular Motion
To switch from sinusoidal to circular movement, update the move_ball() function in ball.py as follows:

x = center_x + radius * math.cos(2 * math.pi * frequency * t)
y = center_y + radius * math.sin(2 * math.pi * frequency * t)

👩‍💻 Author: César Mateo Sánchez Álvarez

Robotics & Embedded Systems Student 💻🤖

A01541805 | Tecnológico de Monterrey
