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
* [![JavaScript](https://img.shields.io/badge/JavaScript-323330?style=for-the-badge&logo=javascript&logoColor=F7DF1E)](https://developer.mozilla.org/en-US/docs/Web/JavaScript)
* [![HTML](https://img.shields.io/badge/HTML-239120?style=for-the-badge&logo=html5&logoColor=white)](https://html.spec.whatwg.org/)
* [![CSS](https://img.shields.io/badge/CSS3-1572B6?style=for-the-badge&logo=css3&logoColor=white)](https://developer.mozilla.org/en-US/docs/Web/CSS)




## Key Features: Unleashing the Robot Mastery

**Real-time Marvel**: Take charge with lightning-fast, real-time control over every move and action of your robot. Say goodbye to delays and embrace the future of instantaneous control!

**Intuitive Wizardry**: Our user interface is not just user-friendly; it's user-magical! Designed to cater to both novices and seasoned pros, it ensures that operating the robot becomes a delightful journey for everyone.

**Effortless Initialization**: Initiate the magic remotely by connecting to your ROS workspace server IP and your local IPv4. No need to be a tech wizard - it's as easy as pie. Just a few clicks, and you're ready to roll!

>[!NOTE]
>Only for the **Bold**: Dive into this adventure only if you've got all the prerequisite packages installed.



## Flow Of Control
![image](https://github.com/atom-robotics-lab/MR-ROBOT-gui/assets/150596140/20af61d9-6df5-4a98-90ce-9b1d25afbc1d)




## Technologies Used

**_Frontend_**: HTML, CSS, JAVASCRIPT

**_Backend_**: JAVASCRIPT

**_Communication Protocols_**: WebSocket, ROSBridge suite


## Getting Started

# ROS Noetic Unleashed: A Step-by-Step Guide

Follow these steps to install ROS Noetic on your Ubuntu 20.04 system and set the stage for an exhilarating technological journey:

 **Step 1: Setup Your Sources.list**
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

 **Step 2: Set Up Your Keys**
Install curl and add the ROS key to your system:
```bash
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

 **Step 3: Update Your Package List**
Before installing ROS, update your system's package list:
```bash
sudo apt update
```

 **Step 4: Install ROS Noetic**
Choose the installation option that suits your needs:

- Full Desktop Install (recommended):
  ```bash
  sudo apt install ros-noetic-desktop-full
  ```

- Desktop Install:
  ```bash
  sudo apt install ros-noetic-desktop
  ```

- ROS-Base:
  ```bash
  sudo apt install ros-noetic-ros-base
  ```

**Step 5: Environment Setup**
Source the ROS setup script in every bash terminal you use ROS in. You can automate this process by adding the following lines to your `~/.bashrc` file:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Step 6: Dependencies for Building Packages**
Install the necessary tools and dependencies for building ROS packages:
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

**Step 7: Initialize rosdep**
Before using many ROS tools, initialize rosdep:
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Gear Up: Prerequisites for the Adventure

To embark on this thrilling journey with MR-ROBOT-gui, make sure your toolkit is armed with the following essentials:

- **Ubuntu 20.04**: The stage for our adventure is set on the robust foundation of [Ubuntu 20.04]("https://releases.ubuntu.com/focal/"). Ensure you have it ready to go.

- **Rosbridge Suite**: The silent conductor orchestrating seamless communication between our UI and the robot's control system. Make sure Rosbridge Suite is in your toolkit. TO install use the following command:
  ``` bash
  sudo apt-get install ros-noetic-rosbridge-server
  ```

- **Web Video Server**: Transforming the visual experience, this server is a must to ensure your visuals are top-notch.
  ```bash
  sudo apt-get install ros-noetic-web-video-server
  ```

- **Simulation Setup**: For the grand simulation, ensure it mirrors the brilliance of [MR-ROBOT](https://github.com/atom-robotics-lab/MR-Robot).


## MR-ROBOT-gui in Action

Follow these steps to initialize MR-ROBOT:

### Step 1: Running MR-ROBOT
```bash
roscore
```

```bash
roslaunch mr_robot_gazebo turtlebot3_house.launch camera_enabled:=true lidar_enabled:=true kinect_enabled:=true
```
The simulation should commence as indicated below:

![image](https://github.com/atom-robotics-lab/MR-ROBOT-gui/assets/150596140/a101ade1-e541-4c70-96c5-664bb7e6fa1c)


```bash
roslaunch mr_robot_nav navigation.launch
```


![rvizpost](https://github.com/atom-robotics-lab/MR-ROBOT-gui/assets/150596140/597ec879-8180-48f8-b3a5-40186940820e)


>[!IMPORTANT]
>Before starting the nav package, ensure the move_base launch file includes the following command for flawless execution:

```bash
<node pkg="robot_pose_publisher" type="robot_pose_publisher" respawn="false" name="posepub"></node>
```




### Step 2: Initializing Web Connectivity
In separate terminals, execute the following commands to set up the web connectivity:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

```bash
rosrun web_video_server web_video_server
```

### Step 3: Open the UI
With MR-ROBOT and the web connectivity ready, open the UI for seamless control and monitoring interface.

## Navigate with Ease: MR-ROBOT-gui User Guide

Unlock the full potential of MR-ROBOT-gui with our user-friendly guide. From power-ups to navigation controls, here's everything you need to know:

![gui1](https://github.com/atom-robotics-lab/MR-ROBOT-gui/assets/150596140/5e0d2c94-a0e3-4f1f-947c-5f19f43a1dae)

### 1. **Power Button:**
   The mighty power button is your gateway to connection bliss. When activated, it establishes the link between the simulation and the webpage through WebSocket magic. Witness the transformation as the UI comes to life.

### 2. **Minimap:**
   Behold the world from a bird's eye view! The minimap gracefully displays the robot's position on the map. Give it a double-click, and it expands into a full-screen marvel, providing a closer look at the robot's journey.

### 3. **Setting Goals:**
   In the realm of the full-screen map, your desires become the robot's goals. Simply double-click on the map to set a goal for the robot. It's not just navigation; it's a command given with a touch.
   
![gui2](https://github.com/atom-robotics-lab/MR-ROBOT-gui/assets/150596140/8e3562ea-7a39-4c11-9a4d-9920072de43b)

### 4. **Keyboard Controls:**
   Engage with the robot using intuitive keyboard controls:
   - Press "W" to move forward.
   - Press "S" to move backward.
   - Press "A" to move left.
   - Press "D" to move right.
   - Press "SHIFT" to increase speed.

   Navigate with the ease of keystrokes and command the robot with precision.

### 5. **Joystick Control:**
   Embrace a new level of control with the joystick. Feel the movement of the robot at your fingertips, steering its path with the fluidity of a maestro.

With this guide in hand, you're not just a user; you're a commander, orchestrating the movements of MR-ROBOT with finesse. Let the exploration begin, and may your commands lead the way!

## Join the Movement: Contribute to MR-ROBOT-gui

Calling all enthusiasts! We invite you to be part of the MR-ROBOT-gui community and contribute to the evolution of this incredible project. Here's how you can get involved:

### Contribution Guidelines:

1. **Fork and Clone:**
   - Fork the repository and clone it to your local machine.

2. **Create a New Branch:**
   - For your feature or bug fix, create a new branch:
     ```bash
     git checkout -b feature/new-feature
     ```
     or
     
     ```bash
     git checkout -b bugfix/issue-number
     ```

3. **Make Changes and Test:**
   - Implement your changes and thoroughly test them.

4. **Commit Your Changes:**
   - Commit your changes with a descriptive message:
     ```bash
     git commit -m "Description of your changes"
     ```

5. **Push to Your Branch:**
   - Push your changes to your branch:
     ```bash
     git push origin feature/new-feature
     ```

6. **Submit a Pull Request:**
   - Submit a pull request to the `main` branch of the original repository.
   - Provide a clear and descriptive title.
   - In the description, explain the purpose of your changes and offer any necessary context.

7. **Follow Coding Conventions:**
   - Ensure your code adheres to the project's coding conventions and style guidelines.

8. **Feedback and Iteration:**
   - Be prepared to address feedback and iterate on your changes if necessary.

9. **Code of Conduct:**
   - Abide by the [GitHub Code of Conduct](https://docs.github.com/en/site-policy/github-terms/github-community-code-of-conduct).

Thank you for joining the movement and contributing to the growth of MR-ROBOT-gui! Your efforts are valued and essential to the success of this project. Let's shape the future of robot control together!


## License
[Apache 2.0]("https://opensource.org/license/apache-2-0/")

## Contact Us

If you have any feedback, please reach out to us at:  
Our Socials - [Linktree](https://linktr.ee/atomlabs)
## Acknowledgments

* [Our wiki](https://atom-robotics-lab.github.io/wiki)
* [ROS Official Documentation](http://wiki.ros.org/Documentation)
* [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
* [Ubuntu Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
