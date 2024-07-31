# My Robot
The purpose of this repository is to design and build a mobile robot, and house it in world. 
Then, program a robot with C++ nodes in ROS to chase a white colored ball!

<img src="go_chase_it.gif"/>

## Description
Inside the Gazebo world one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.
* A white ball that is to be followed by the robot.

## Getting Started

### Directory structure
    .GoChaseIt                              # Go Chase It Project
    ├── my_robot                            # my_robot package                   
    │   ├── launch                          # launch folder for launch files   
    │   │   ├── robot_description.launch    # Generate urdf from xacro
    │   │   ├── world.launch                # launch Gazebo world along with robot
    │   ├── meshes                          # meshes folder for sensors
    │   │   ├── hokuyo.dae                  # Hokuyo lidar sensor
    │   ├── urdf                            # urdf folder for xarco files
    │   │   ├── my_robot.gazebo             # Robot description
    │   │   ├── my_robot.xacro              # Plugin for sensor/actuator (Camera/Hokuyo lidar/Differential drive)
    │   ├── world                           # world folder for world files
    │   │   ├── office.world
    │   ├── CMakeLists.txt                  # compiler instructions
    │   ├── package.xml                     # package info
    ├── ball_chaser                         # ball_chaser package                   
    │   ├── launch                          # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                             # source folder for C++ scripts
    │   │   ├── drive_bot.cpp               # Node to command wheel joint velocities to robot
    │   │   ├── process_images.cpp          # Node to request drive_bot in the direction of ball
    │   ├── srv                             # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt                  # compiler instructions
    │   ├── package.xml                     # package info                  
    └──                          

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

### How to Run
* Update and upgrade the Workspace
```
sudo apt-get update && sudo apt-get upgrade -y
```
* Create a [catkin workspace](https://wiki.ros.org/catkin/conceptual_overview)
```
$ mkdir -p ~/catkin_ws/src
```
* Navigate to source directory
```
$ cd ~/catkin_ws/src
```
* Initialize the catkin workspace which will create a ```CMakeLists.txt``` file.
```
catkin_init_workspace
```
* Clone this repository.
```
git clone https://github.com/sidharth2189/RoboND-GoChaseIt.git
```
* Copy ```my_robot``` and ```ball_chaser``` packages into the source folder for catkin workspace.```/catkin_ws/src```
* Navigate to catkin workspace.
```
cd ~/catkin_ws/
```
* Build packages.Note that the command is issued from within the top level directory (i.e., within ```catkin_ws``` NOT ```catkin_ws/src```) 
```
catkin_make
```
* Source the set up script of the workspace. 
```
source devel/setup.bash
```
* To check for missing package.
```
rosdep check <package name>
```
* Launch the robot inside the world.
```
roslaunch my_robot world.launch
```
* Run ```drive_bot``` and ```process_image``` nodes in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

## Useful links

* [Gazebo laser sensor](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Laser)
* [Gazebo camera sensor](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera)
* [Lidar plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/RayPlugin.cc)
* [Camera plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/CameraPlugin.cc)
* [Differential drive actuator plugin](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/DiffDrivePlugin.cc)
* [Mesh files for entire library of models in Gazebo](http://models.gazebosim.org/)
