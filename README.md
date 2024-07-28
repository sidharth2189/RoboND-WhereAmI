# My Robot
The purpose of this repository is to design and build a mobile robot, and house it in world. Then, program a robot with C++ nodes in ROS to chase white colored balls!

## Description
Inside the Gazebo world one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.

## Getting Started

### Directory structure
    .GoChaseIt                         # Build Gazebo World Project 
    ├── urdf                           # Robot definition 
    │   ├── my_robot.xacro             # Robot description
    │   ├── my_robot.gazebo            # Plugin for sensor/actuator (Camera/Hokuyo lidar/Wheel joints)
    ├── launch                         # Launch nodes      
    │   ├── world.launch               # Gazebo world along with robot
    │   ├── robot_description.launch   # Generate urdf from xacro
    ├── meshes                         # Mesh file for Hokuyo lidar sensor      
    │   ├── hokuyo.dae
    ├── worlds                         # Gazebo main World containing models 
    │   ├── empty.world
    ├── CMakeLists.txt                 # Link libraries 
    └── package.xml                          

### Dependencies

* Operating System — Ubuntu Bionic 18.04 LTS or 20.04 LTS (Focal Fossa), and WSL on Windows
* Software packages — CMake 2.8 or later, ROS Noetic, Gazebo 11
    * [Gazebo Classic 11.0](https://classic.gazebosim.org/) will reach its end of life by Feb 2025.
    * [ROS Noetic](https://wiki.ros.org/noetic) is supported for 20.04 and will reach its end of life in May 2025.

### Installing

* [Install ROS Noetic using Ubuntu/WSL](https://wiki.ros.org/noetic/Installation/Ubuntu)
* [Install Gazebo using Ubuntu packages](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
* [Install ROS Noetic using Robostack for Mac](https://robostack.github.io/GettingStarted.html)
* [Install Gazebo on Mac](https://classic.gazebosim.org/tutorials?tut=install_on_mac&cat=install)
* To verify installation, run
```
gazebo
```

## Useful links

* [Gazebo laser sensor RRBot example](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Laser)
* [Gazebo camera sensor RRBot example](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera)
* [Lidar plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/RayPlugin.cc)
* [Camera plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/CameraPlugin.cc)
* [Wheel joint actuator plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/CameraPlugin.cc)
