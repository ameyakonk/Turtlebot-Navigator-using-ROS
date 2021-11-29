# turtlebot_navigator

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Project Description

The program launches Turtlebot3 in Turtlebot3_Gazebo world and navigates autonomously by detecting
and avoiding bostacles

## Personnel

### Ameya Konkar 

UID:118191058

Master's Student at University of Maryland, College Park

## Overview

### Dependencies
This is a ROS package which needs [ROS Noetic](http://wiki.ros.org/Installation/Ubuntu) to be installed on Ubuntu 20.04.

### Building the Program and Tests

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ameyakonk/turtlebot_navigator.git
cd ..
catkin_make

```
### Initiating roscore

Open Terminal
```
roscore
```

### Run program with a launch file

Open Terminal
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch turtlebot_navigator turtlebot_navigation.launch

```

## ROSBAG Recording
The published topics can be saved into a rosbag. The launch file has an argument to enable recording of the topics into a rosbag. To run the nodes and record the published topics, execute the folowing command in a terminal:
```
roslaunch turtlebot_navigator turtlebot_navigation.launch rosbagRecord:=true
``` 
To view the rosbag, run the commands
```
cd <path to catkin_ws>/src/turtlebot_navigator/results
rosbag play navigator_record.bag
```
