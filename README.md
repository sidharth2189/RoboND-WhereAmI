# My Robot
The purpose of this repository is to estimate a robot's position relative to a known map of environment using [adaptive monte carlo localization](https://wiki.ros.org/amcl). 

The steps are listed as [summary of tasks](task_summary.txt).

<img src="amcl.gif"/>

## Description
Inside the Gazebo world one can identify:

* Two wheeled Robot with caster.
* Sensors (lidar and camera) mounted on the robot.

## Getting Started

### Directory structure
    .WhereAmI                               # Robot localization Project
    ├── my_robot                            # my_robot package                   
    │   ├── launch                          # launch folder for launch files   
    │   │   ├── robot_description.launch    # Generate urdf from xacro
    │   │   ├── world.launch                # launch Gazebo world along with robot
    │   │   ├── amcl.launch                 # launch robot localization using amcl    
    │   ├── meshes                          # meshes folder for sensors
    │   │   ├── hokuyo.dae                  # Hokuyo lidar sensor
    │   ├── urdf                            # urdf folder for xarco files
    │   │   ├── my_robot.gazebo             # Plugin for sensor/actuator (Camera/Hokuyo lidar/Differential drive)
    │   │   ├── my_robot.xacro              # Robot description
    │   ├── world                           # world folder for world files
    │   │   ├── office.world
    │   ├── CMakeLists.txt                  # compiler instructions
    │   ├── package.xml                     # package info
    │   ├── config                          # parmater for robot's navigational goal   
    │   │   ├── costmap_common_params.yaml  # rosparam for move_base package
    │   │   ├── local_costmap_params.yaml   # rosparam for move_base package
    │   │   ├── global_costmap_params.yaml  # rosparam for move_base package
    │   │   ├── base_costmap_params.yaml    # rosparam for move_base package
    │   ├── maps                            # parmater for robot's navigational goal   
    │   │   ├── map.pgm                     # map generated from pgm_map_creator package
    │   │   ├── map.yaml                    # map metadata    
    ├── pgm_map_creator                     # map creator package (submodule)
    ├── teleop_twist_keyboard               # control robot motion through keyboard (submodule)  
    ├── amcl.rviz                           # visualization file                                      
    └──                          

### Dependencies

* Operating System — Ubuntu 16.04 LTS. ([Udacity VM Image](https://s3-us-west-1.amazonaws.com/udacity-robotics/Virtual+Machines/Lubuntu_071917/RoboVM_V2.1.0.zip))
    *  Please refer steps for usage of VM, resource allocation and first boot [here](/docs/VM.txt).

### Installing
* To verify installation, run
```
gazebo
```

### How to generate map from gazebo world environment
* The ROS amcl node uses a [map file](/docs/pgm_map).
* This step helps generate a [map](/my_robot/maps/map.pgm) for the robot to knows what to expect in environment.
* For this purpose, [pgm_map_creator](/pgm_map_creator/) is used.
* Navigate to ROS package folder and create a maps folder. That's where the map file will reside.
```
cd /home/workspace/catkin_ws/src/<YOUR PACKAGE NAME>
```
```
mkdir maps
```
* Install Dependencies for compiling map creator.
```
sudo apt-get install libignition-math2-dev protobuf-compiler
```
* Clone repository or use the [submodule](/pgm_map_creator/) in this repository.
```
cd /home/workspace/catkin_ws/src/
```
```
git clone https://github.com/hyfan1116/pgm_map_creator.git
```
* Build package
```
cd ..
catkin_make
```
* Add and Edit the World File
Copy the Gazebo world you created to the world folder in pgm_map_creator
```
cp <YOUR GAZEBO WORLD FILE> src/pgm_map_creator/world/<YOUR GAZEBO WORLD FILE>
```
* Insert the map creator plugin to world file. Open the world file using the editor of your choice. Add the following tag towards the end of the file, but just before </world> tag:
```
<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
```
* Create the PGM map
Open a terminal, run gzerver with the map file
```
gzserver src/pgm_map_creator/world/<YOUR GAZEBO WORLD FILE>
```
* Open another terminal, launch the request_publisher node
```
roslaunch pgm_map_creator request_publisher.launch
```
* Wait for the plugin to generate map. It will be located in the map folder of the pgm_map_creator! 
* Open it to do a quick check of the map. If the map is cropped, you might want to adjust the parameters in launch/request_publisher.launch, namely the x and y values, which defines the size of the map
```
  <arg name="xmin" default="-15" />
  <arg name="xmax" default="15" />
  <arg name="ymin" default="-15" />
  <arg name="ymax" default="15" />
  <arg name="scan_height" default="5" />
  <arg name="resolution" default="0.01" />
```
* Edit the Map
If map is not accurate due to the models, feel free to edit the pgm file directly!

* Add the Map to robot Package
Now we have the map file, let us move it to where it is needed! That is the maps folder created at the very beginning of [robot package](/my_robot/)
```
cd /home/workspace/catkin_ws/
cp src/pgm_map_creator/maps/<YOUR MAP NAME>  src/<YOUR PACKAGE NAME>/maps/<YOUR MAP NAME>
```

* A yaml file providing the [metadata about the map](https://wiki.ros.org/map_server#YAML_format). Create a yaml file next to map.
```
cd src/<YOUR PACKAGE NAME>/src/maps
touch <YOUR MAP NAME>.yaml
```

* Add below lines to the yaml file.
```
image: <YOUR MAP NAME>
resolution: 0.01
origin: [-15.0, -15.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

* Note that the origin of the map should correspond to map's size. For example, the default map size is 30 by 30, so the origin will be [-15, -15, 0], i.e. half the size of the map.

### How to run localization
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
* Clone this repository and its submodules.
```
git clone https://github.com/sidharth2189/RoboND-WhereAmI.git
```
```
git submodule update --init --recursive
```
* Copy ```my_robot```, ```pgm_map_creator``` and ```teleop_twist_keyboard``` packages into the source folder for catkin workspace.```/catkin_ws/src```
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
* Launch the robot inside the world. Alongside Gazebo, this also open rviz for visualization.
```
roslaunch my_robot world.launch
```
* Launch amcl in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch my_robot amcl.launch
```
* To visualize the map and robot localization load [amcl.rviz](/amcl.rviz) using rviz window.

### How to tune parameters for localization
* Reference for tuning [amcl parameters](/docs/amcl_parameters.txt) for better results.
* Reference for [move_base parameters](/docs/move_base.txt).

### How to test localization
There are two options to test localization.
* Send navigation goal via RViz.
    * In rviz, click the ```2D Nav Goal``` button in the toolbar, then click and drag on the map send the goal to the robot. It will start moving and localize itself in the process. Refer [clip](/amcl.gif).
    * The amcl node can also be given a nudge, by providing the robot an initial position estimate on the map using ```2D Pose Estimate```.
        * Alternatively, set initial pose for robot in ```amcl.launch``` using below [steps](https://knowledge.udacity.com/questions/343189).
            * Launch the amcl.launch, the map will appear in the RVIZ section.
            * Keep the fixed frame as map and use ```2D pose Estimate``` button to set the robot location such that the laserscan matches the map.
            * Once you finalize the location, you can see it on the terminal where you launched the amcl.launch file, the location will appear which you can use as the initial pose for amcl node. 

* Send move command via ```teleop``` package.
    * Control robot through keyboard as done in [EKF lab](https://github.com/sidharth2189/RoboND-EKFLab). The [teleop node](/teleop_twist_keyboard/teleop) needs to be added to package in this case. 

## Useful links
* [move_base](https://wiki.ros.org/move_base) can define a navigation goal position for your robot in the map, and the robot will navigate to that goal position. Note that this package is optional if [teleop node](https://github.com/ros-teleop/teleop_twist_keyboard) is used instead to send command for robot movement, using keyboard.
* [config files](https://s3-us-west-1.amazonaws.com/udacity-robotics/Resource/where_am_i/config.zip) for parameters for move_base package.
* [map_server](https://wiki.ros.org/map_server) node provides map data as a ROS service to other nodes such as the amcl node. Here, map_server node will locate the map you created in the Map Setup(opens in a new tab) step and send it out as the map data.
* [amcl](https://wiki.ros.org/amcl) package is used in this project for localization.
* [Robot reference](https://github.com/sidharth2189/RoboND-GoChaseIt)
