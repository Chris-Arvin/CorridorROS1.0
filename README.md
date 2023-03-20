# CorridorROS1.0
This is a simulation platform especially for testing robotic motion planning algorithms in complex corridor with crowds. 
An improved Social Form Model VSFM is assembled to drive pedestrians. Several challenging corridor scenarios are provided. 
Six motion planning baselines are adapted. Two robot models are designed. 

[![](https://user-images.githubusercontent.com/36269534/226112301-6a9947d6-79c9-4ed5-a1df-e17ba4848b7b.png)](https://youtu.be/v-9p0tqyNOM "")

* The pedestrian simulation is extended from [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros). We complete it, providing three optional person drivers: data replay, extended social force model, and manual control.
* The navigation simulation is extended from [move_base](https://github.com/ros-planning/navigation), a powerful ROS navigation stack. We simplify some existing algorithms and adapt them to the corridor scenarios. 


## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)
* [Introduction for Key Parameters and Topics](#3-Introduction-for-Key-Parameters-and-Key-Topics)
* [More Examples](#4-More-Examples)
* [How to DIY](#5-Instructions-to-DIY)
* [Contributors](#6-Contributors)
* [Acknowledgement](#7-Acknowledgement)


## 1. Installation
The project has been tested on Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic). We highly recommend using Ubuntu 18.04 since Ubuntu 16.04 will no longer be supported after April 2021. In the following we will take ROS Melodic version as the example. Please install some dependence firstly: 
```
$ sudo apt install ros-melodic-navigation
```
Then please install this project and build it: 
```
$ mkdir -p CorridorROS_ws/src
$ cd CorridorROS_ws/src
$ git clone https://github.com/Chris-Arvin/CorridorROS1.0.git
$ cd ..
$ catkin_make
```

## 2. Quick Start
Please open a terminal to launch the pedestrian simulation: 
```
$ source CorridorROS_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=1 robot_mode:=0
```
open another terminal to launch the navigation simulation: 
```
$ source CorridorROS_ws/devel/setup.bash
$ roslaunch move_base_bridge move_base_bridge.launch
```
Currently, the pedestrians are drven by VSFM, and the robot is driven TEB. 


## 3. Introduction for Key Parameters and Key Topics
#### 3.1 Parameters in "pedsim_simulator.launch"
@param: robot_model
* water: a two-wheel differential mobile robot
* wheeltec: a car-like robot

@param: person_mode
* 0: drive the pedestrian with data replay
* 1: drive the pedestrian with extended social force model
* 2: drive the pedestrian with manual control

@param: robot_mode
* 0: drive the robot with algorithms (baselines or your own algorithm) in the format of the plugin
* 1: drive the robot with extended social force model
* 2: drive the robot with manual control

@param: scene_file
* the localization of a .xml file descriping the obstalce distribution, pedestrian distribution, and the robot state. 


#### 3.2 Parameters in "move_base.launch"
@param: local_plan
* [TEB](https://github.com/rst-tu-dortmund/teb_local_planner): Timed Elastic Band
* [DWA](https://github.com/amslabtech/dwa_planner): Dynamic Window Approach
* [Bezier](https://github.com/marinaKollmitz/lattice_planner): generate paths with Bezier curve, and trace the path with PID controller
* [TBL](https://github.com/marinaKollmitz/human_aware_navigation): a simplified version of Time Bounded Lattice
* [PGL](https://ieeexplore.ieee.org/document/9981332): an improved version of TBL, considering the interaction between the robot and pedestrians as a participant game mode. 
* [MPC](https://github.com/JunshengFu/Model-Predictive-Control): Model Predictive Control

#### 3.3 Topics about the environment
@topic: /map
* do not project the person into the costmap, only including the static obstacles
@topic: /map_with_people
* project the person into the costmap, regarding the persons as dynamic obstacles
@topic: /persons
* the state of the persons, including their pose and velocity


## 4. More Examples to open "pedsim_simulator.launch"
Change the params "person_mode" or "robot_mode" in [Quick Start](#2-Quick-Start) to use the simulation platform variously:

#### Case1: drive the person with data replay
open two terminals and input the commands separately: 
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=0
$ rosbag play $(file_name.bag) /persons:=/persons_recorded
```
note:
* You can easily create your own recorded data according to [Instructions to DIY](#5-Instructions to DIY)
#### Case2: drive the person with improved Social Force Model
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=1
```
#### Case3: drive the person with keyboard
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2
```
note: 
* comment the third line and uncomment the fourth line in "interface_teleop.launch" to control the pedestrian one by one: move the pedestrian with "t", "f", "g", "h", "b", and change the ID with "j" or "k".
* uncomment the third line and comment the fourth line in "interface_teleop.launch" to control a few pedestrians simutanuously. Due to the small number of keys, up to 3 people can be controlled at the same time. 
* if "enable_gaze_control" in "pedsim_simulator.launch" is true, control the human gaze with "8", "4", "6", "2", and change ID with "5". 

#### Case4: drive the robot with provided algorithms
```
$ roslaunch pedsim_simulator pedsim_simulator.launch robot_mode:=0
```
note: 
* You can change the baseline in **move_base.launch**, please change the parameter "local_plan" according to guidance. 
* Also, you can adapt your own algorithm in the format of the plugin. The details about the plugin can be found at [how to create a plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
#### Case5: drive the robot with VSFM
```
$ roslaunch pedsim_simulator pedsim_simulator.launch robot_mode:=1
```
#### Case6: drive the robot with keyboard
```
$ roslaunch pedsim_simulator pedsim_simulator.launch robot_mode:=2
```
note: 
* move the robot with "w", "a", "s", "d", "x". 


## 5. Instructions to DIY

#### 5.1 DIY for simulation environment
We allow users to build its simulation environment in the format of xxx.xml. A reference xml file can be found at **example_env.xml**. 
Slam for the environment is unnecessary, because we bridge the auto map-sending function. 
In .xml file, four parts mush be specified： 
```
<!--Obstacles-->
<!--Potential targets of pedestrians-->
<!--AgentClusters-->
<!--Robot-->
```
Use your environment by replacing the source of your **scene_file** in **pedsim_simulator.launch**

#### 5.2 Create a new recorded data
Firstly, open the environment and manually control persons according to [Quick Start](#2-Quick-Start).
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=0
```
Next, open another terminal to record the person states:
```
$ rosbag record /persons -o $(scene_file.bag)
```

To drive persons with recorded data, open two terminals separately according to [Case1](#Case1-drive-the-person-with-data-replay): 
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=0
$ rosbag play $(scene_file.bag) /persons:=/persons_recorded
```


## 6. Contributors
* Qianyi Zhang  zhangqianyi@mail.nankai.edu.cn
* Yinuo Song
* Zhuoran Wang
* Jiahang WU
* Jingtai Liu


## 7. Acknowledgement
These packages have been developed from [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) and [move_base](https://github.com/ros-planning/navigation). Thanks again to support our work.

