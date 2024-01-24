# MR-ROBOT-gui
A ros and javascript based Web Ui for controlling Mr-robot.

## ABOUT THIS PROJECT
The ROBOT-gui is a user interface designed to facilitate the remote control and monitoring of ros based robots. Developed with the goal of providing an intuitive and efficient control experience, this web application enables users to interact with the robot's functionalities in real-time through a web browser.

## BUILT WITH
### Built With

* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)
* [![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
* [![MediaPipe](https://img.shields.io/badge/mediapipe-%4285F4.svg?style=for-the-badge&logo=mediapipe&logoColor=white)](https://developers.google.com/mediapipe)




## Key Features

**Real-time Control** : Experience seamless control over the robot's movements and actions with minimal latency.

**Intuitive Interface** : The user interface is designed to be user-friendly, allowing both novice and experienced users to operate the robot effectively.

**Easy Initialization** : Can be initialized remotely using your ros workspace server ip and your local IPv4 

[!NOTE]


## Technologies Used

**_Frontend_**: HTML, CSS

**_Backend_**: Javascript

**_Communication Protocol_**: WebSocket, ROSBridge suite

## How It Works
The web UI communicates with the robot's control system through the communication protocols, establishing a bidirectional channel for transmitting commands and receiving feedback. The frontend interface provides a comprehensive set of controls and visualizations, allowing users to monitor the robot's position and the inbuilt camera's feed to make real-time adjustments to its operations.

## Getting Started

## Prerequisites

- Ubuntu 20.04

- [Ros noetic](https://wiki.ros.org/noetic), Gazebo 11 and Rviz

  For getting started with ros noetic refer to [Atom Robotics Lab Wiki](https://atom-robotics-lab.github.io/wiki/markdown/ros/ROS_installation/ROS_index.html)

- [Rosbridge Suite](https://wiki.ros.org/rosbridge_suite)

  `sudo apt-get install ros-noetic-rosbridge-server`

- [Web Video Server](https://wiki.ros.org/web_video_server)

  For installation use the [github repository](https://github.com/RobotWebTools/web_video_server)

- The simulation to be run similar to [MR-ROBOT](

  

## Installation
1. Make a new workspace



### Usage

Installation


bash
Copy code

# Example installation command
npm install my-package
Usage
Provide examples and explanations of how to use your project. This can include code snippets, screenshots, or gifs to demonstrate functionality.

javascript
Copy code
// Example usage code
const myPackage = require('my-package');
Contributing
Explain how others can contribute to your project. This can include guidelines for submitting bug reports, feature requests, or code contributions.

License
Specify the license under which your project is released. For example:

This project is licensed under the MIT License - see the LICENSE.md file for details.

Acknowledgements
If your project uses third-party libraries, tools, or resources, you can acknowledge them here.

Additional Sections
Depending on the nature of your project, you might want to include additional sections such as:

Documentation: Links to further documentation or a dedicated documentation section.
Changelog: Information about changes in each version of the project.
Support: Information on how users can get support for your project.
Roadmap: Plans for future development of the project.
