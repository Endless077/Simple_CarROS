![Logo](https://github.com/Endless077/Simple-CarROS/blob/main/ros_logo.png)

# CarROS 🚗

CarROS is a project developed in ROS Noetic on Ubuntu 20.04 that simulates a small autonomous car using Gazebo and RViz. The car can navigate autonomously by avoiding obstacles or be manually controlled, providing an ideal environment to experiment with and learn robotics and autonomous navigation concepts.


## 🔑 Key Features

- **Extensibility**: Designed to be easily extended with new features or sensors.
- **ROS Integration**: ROS-based implementation leveraging nodes, services, topics, and parameters for modular management.
- **Simulation**: Compatibility with Gazebo for advanced simulation and RViz for visualization.
- **Autonomous Mode**: Autonomous mode with obstacle avoidance using virtual sensors.
- **Manual Control**: Manual control via a GUI or direct commands.


## 🛠️ Installation

### 📋 Prerequisites

Make sure you have installed:

- **ROS Noetic**: Follow the [official guide](http://wiki.ros.org/noetic/Installation/Ubuntu).
- **Gazebo**: Pre-installed with ROS, but you can verify with:
  ```bash
  sudo apt install gazebo
  ```
- **RViz**: Included in ROS Desktop Full.

### ⚙️ Installation Steps

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Endless077/Simple_CarROS.git
   cd CarROS
   ```

2. **Build the Project**:
   - Navigate to the project's workspace and build using `catkin_make`:
     ```bash
     catkin_make
     ```

3. **Source the Environment**:
   - Source the development environment:
     ```bash
     source devel/setup.bash
     ```

4. **Gazebo Dependencies**:
   - At first launch Gazebo may require additional dipenences
     
5. **Run the Simulation**:
   - Launch an Autonomous Simulation with:
     ```bash
     roslaunch ros_car_cmd ros_car_autonomus.launch
     ```
   - Launch a Manual Simulation with:
     ```bash
     roslaunch ros_car_cmd ros_car_manual.launch
     ```
     
### ⚠️ Notes

- For the best experience, use Ubuntu 20.04 with ROS Noetic Desktop Full.
- Customize your simulation with all the params files in ros_car_cmd packaage.


## 📜 API Reference

The project leverages ROS's modular architecture with:

- **Nodes**: For managing control and navigation functionalities.
- **Services**: For high-level commands like switching between autonomous and manual modes.
- **Topics**: For communication of sensor data and movement commands.
- **Parameters**: For dynamic configurations during runtime.

Refer to the auto-generated ROS documentation for a detailed list with this (package)[https://wiki.ros.org/rosdoc_lite].


## 🙏 Acknowledgements

### Gazebo 🌍

Gazebo is a robotic simulation tool that allows testing algorithms and models in a realistic environment.

[More information here](http://gazebosim.org/).

### RViz 🎨

RViz is a visualization tool for ROS that shows sensor data and robot states in real time.

[More information here](http://wiki.ros.org/rviz).

### ROS_LIB.js 📚

A JavaScript library for interfacing with ROS via WebSocket, useful for creating interactive GUIs for manual control.

[More information here](http://wiki.ros.org/roslibjs).

### Robot Operating System (ROS) 🤖

ROS is an open-source framework for robotics that provides tools and libraries to create complex robotic applications. CarROS uses nodes, topics, services, and parameters to achieve a modular and scalable system.

[More information here](http://wiki.ros.org/).


## 💾 License

This project is licensed under the GNU General Public License v3.0.

[GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html)

![Static Badge](https://img.shields.io/badge/UniSA-CarROS-red?style=plastic)


## 🖐 Author

**Contributors:**
- No contributors

**Project Manager:**
- [Antonio Garofalo](https://github.com/Endless077)


## 🔔 Support

For support, email [antonio.garofalo125@gmail.com](mailto:antonio.garofalo125@gmail.com) or contact the project contributors.


### 📝 Documentation

See the documentation project **[here](https://github.com/Endless077/Simple_CarROS/blob/main/docs.pptx)**.
