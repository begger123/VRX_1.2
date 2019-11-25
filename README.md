# Documentation on an academic project you say???
Here lies the source code for Florida Atlantic University's VRX 2019 effort.

## Team Members
Eric Nieves
Armando Sinisterra
Daniel Resio
Alex Barker
Adam Hall
Travis Moscicki

## System Dependencies
For the VRX Simulation:  
`sudo apt update`
`sudo apt full-upgrade`
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
`sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'`
`wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -`
`sudo apt update`
`DIST=melodic`
`GAZ=gazebo9`
`sudo apt install cmake mercurial git ruby libeigen3-dev ${GAZ} lib${GAZ}-dev pkg-config python ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-hector-gazebo-plugins ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-key-teleop ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-rviz ros-${DIST}-ros-base ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-keyboard ros-${DIST}-velodyne-simulator ros-${DIST}-xacro ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins protobuf-compiler`   
For Owltonomous base code:  
`sudo apt-get install ros-melodic-pointcloud-to-laserscan`  
*more to be added


A whole bunch...
Ubuntu 18.04
Ros Melodic
### Add specific ros packages
Python
### Add specific python packages
C++11

## System Hardware and Terminology

## System Structure
The code base is organized into the following main folders.  Note that the dependencies are listed above in totality, but the specific dependenices of each subsystem are listed accordingly.
More information is provided in the top level README of each of these subfolders.
### communicationSystems
Stores code that supports the vehicle's communication with the competition server du jour.

### Debugging

### highLevelSystems
Stores the state machine (missionPlannerBiggie), usbl, usv16_blm_extra, and visionSystemsBiggie.  
*missionPlannerBiggie is a collection of classes in C++ that allows for a [hopefully] modular approach to autonomy programming
*usbl is the Jetson side interface for the STM32 USBL dev board.  It takes the processed USBL data and converts to ROS
*usv16_blm_extra is an interface level for the bottom level manager (blm).  This board peforms health monitoring, including bus voltages and current draws
*vissionSystemsBiggie houses the perception system.   

### lowLevelSystems
Stores lowLevelBiggie, vehicleSupportBiggie, and wamv_navigation


### vrx
=======
# VRX_1.2
This is the latest version of the VRX 2019 package including our stack

In terminal, type:
roslaunch vehicle_control sim_pid_control.launch

In RViz:
click anywhere in the map using the 2D Nav Goal tool and see the path-planner in action
while the vehicle moves

This branch explores the implementation of quadratic programming (QP) optimization technique for thrust and azimuth allocation on both port and starboard thrusters.
