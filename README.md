# MR-ROBOT-gui: Unleash the Power of Control!

Welcome to the vibrant world of MR-ROBOT-gui, where ROS and JavaScript converge to create a seamless Web UI for taking charge of Mr. Robot.

## EMBRACE THE INNOVATION

Dive into the heart of this project and witness the extraordinary. The ROBOT-gui stands as a testament to the fusion of cutting-edge technology, blending ROS and JavaScript to bring you a web interface that redefines the way you control and monitor ROS-based robots.

## EXPERIENCE CONTROL LIKE NEVER BEFORE

Say goodbye to the mundane, and welcome an interface that goes beyond expectations. Our web application is not just about remote control; it's about an immersive and dynamic interaction with your robot. Real-time control and monitoring are no longer a chore; they're an exhilarating experience.

## INTUITIVE DESIGN, EFFICIENT CONTROL

Navigate effortlessly through our thoughtfully crafted user interface. The goal? To provide you with an intuitive and efficient control experience. MR-ROBOT-gui ensures that controlling your robot is not just a task but a joyous journey.

## REAL-TIME INTERACTION, ANYTIME, ANYWHERE

Thanks to MR-ROBOT-gui, the power to interact with your robot is now at your fingertips, accessible through any web browser. Whether you're at home, in the office, or on the go, the connection remains seamless, and the control remains in your hands.

Get ready to revolutionize your robot control experience. MR-ROBOT-gui: Elevating control to a whole new level!


## BUILT WITH


* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)
*[![JavaScript](https://img.shields.io/badge/logo-javascript-blue?logo=javascript)](https://developer.mozilla.org/en-US/docs/Web/JavaScript)

*[![HTML](https://img.shields.io/w3c-validation/:parser)](https://html.spec.whatwg.org/)

*[![CSS](![Static Badge](https://img.shields.io/badge/:CSS))](https://developer.mozilla.org/en-US/docs/Web/CSS)



## Key Features: Unleashing the Robot Mastery

**Real-time Marvel**: Take charge with lightning-fast, real-time control over every move and action of your robot. Say goodbye to delays and embrace the future of instantaneous control!

**Intuitive Wizardry**: Our user interface is not just user-friendly; it's user-magical! Designed to cater to both novices and seasoned pros, it ensures that operating the robot becomes a delightful journey for everyone.

**Effortless Initialization**: Initiate the magic remotely by connecting to your ROS workspace server IP and your local IPv4. No need to be a tech wizard - it's as easy as pie. Just a few clicks, and you're ready to roll!

[!ATTENTION]
Only for the Bold: Dive into this adventure only if you've got all the prerequisite packages installed. But trust us, the thrill is worth it!


## Technologies Used

**_Frontend_**: HTML, CSS, JAVASCRIPT

**_Backend_**: JAVASCRIPT

**_Communication Protocol_**: WebSocket, ROSBridge suite

## Unveiling the Mechanism

Prepare to be dazzled as we unravel the enchanting dance between the web UI and Mr. Robot's control system. Here's a glimpse into the magic:

**1. Communication Choreography:**
   The web UI and the robot's control system engage in a mesmerizing dance through communication protocols. Imagine a bidirectional channel where commands flow seamlessly, and feedback gracefully returns.

**2. Frontend Symphony:**
   The frontend interface is not just a visual feast; it's a symphony of controls and visualizations. Delve into a comprehensive set of tools that empower you to monitor the robot's position and gaze through its inbuilt camera's lens. It's not just a UI; it's a canvas for real-time adjustments and control.

**3. Real-time Alchemy:**
   Watch in awe as the web UI transforms into a conduit for real-time adjustments. The robot's every move becomes a reflection of your commands, and the inbuilt camera's feed opens a window into its world, allowing you to fine-tune operations on the fly.

Get ready to be part of the magic show where technology and control come together in perfect harmony. This is not just how it works; it's how it mesmerizes!


## Getting Started

## Prerequisites

- Ubuntu 20.04

- [Ros noetic](https://wiki.ros.org/noetic), Gazebo 11 and Rviz

  For getting started with ros noetic refer to [Atom Robotics Lab Wiki](https://atom-robotics-lab.github.io/wiki/markdown/ros/ROS_installation/ROS_index.html)

- [Rosbridge Suite](https://wiki.ros.org/rosbridge_suite)

  

- [Web Video Server](https://wiki.ros.org/web_video_server)

  For installation use the [github repository](https://github.com/RobotWebTools/web_video_server)

- The simulation to be run similar to [MR-ROBOT](https://github.com/atom-robotics-lab/MR-Robot)

  

## Installation

### (1) Installing ROS Noetic

To install ROS Noetic on Ubuntu 20.04, follow these steps:

1. **Setup your sources.list**
   
   This step involves setting up your computer to accept software from packages.ros.org. Run the following command:

   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

2. **Set up your keys**

   Install curl if you haven't already done so, then add the ROS key to your system:

   ```bash
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

3. **Update your package list**

   Before installing ROS, you should update your system's package list:

   ```bash
   sudo apt update
   ```

4. **Install ROS Noetic**

   There are three options for installing ROS Noetic depending on your needs:

   - Full Desktop Install (recommended): This includes everything in Desktop plus 2D/3D simulators and 2D/3D perception packages. Use the following command:

     ```bash
     sudo apt install ros-noetic-desktop-full
     ```

   - Desktop Install: This includes everything in ROS-Base plus tools like rqt and rviz. Use the following command:

     ```bash
     sudo apt install ros-noetic-desktop
     ```

   - ROS-Base: This is a bare bones installation including ROS packaging, build, and communication libraries. No GUI tools are included. Use the following command:

     ```bash
     sudo apt install ros-noetic-ros-base
     ```

5. **Environment setup**

   You must source this script in every bash terminal you use ROS in. You can automate this process by adding the following lines to your `~/.bashrc` file:

   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

6. **Dependencies for building packages**

   Install the necessary tools and dependencies for building ROS packages:

   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

7. **Initialize rosdep**

   Before you can use many ROS tools, you will need to initialize rosdep. Run the following commands:

   ```bash
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```

After completing these steps, you should have ROS Noetic installed on your Ubuntu 20.04 system.

### (2) Installing Pre-Requisits

Before using Mr.Robot there are a few more prerequisits:
1. **Installing Web Socket**
```bash
  sudo apt-get install ros-noetic-rosbridge-server
  ```



## USAGE
  We've put our UI through its paces on the extraordinary [MR-ROBOT](https://github.com/atom-robotics-lab/MR-Robot), a creation of sheer brilliance from the Atom Robotics Lab.

   1. For running MR-ROBOT use the following commands:

   - ```bash 
   roscore
   ```
   
- ```bash
   roslaunch mr_robot_gazebo turtlebot3_house.launch camera_enabled:=true lidar_enabled:=true kinect_enabled:=true
   ```

- ```bash
   roslaunch mr_robot_nav navigation.launch
   ```

   2. After the sim has been initialized, Run the following commands in seperate terminals

- ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

- ```bash
   rosrun web_video_server web_video_server
   ```

- 







