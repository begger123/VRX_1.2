# missionPlanner #

This repo stores the high level planner currently used on the Florida Atlantic University 16' WAM-V.

At this time, this package is managed by [Travis Moscicki](travismoscicki@gmail.com) (Florida Atlantic University).

## Overview ##
The mission planner package is initiated by running `roslaunch the_planner the_planner.launch`. 
	This will create a `highLevelPlanner` object called `thePlanner` which will be responsible for passing from mission to mission.  A mission list is loaded from line 5 of the `the_planner.launch` that will specify which missions should be run. This allows for flexibility in testing and running the vehicle during demonstration. The available missions are:

* sample_mission
* listen_to_gui
* tester

Each of these missions have a starting position (GPS point and heading) and a timeout specified in `the_planner/yaml/missions.yaml`.

Each mission has a class implemented in `the_planner/src/missions` that inherits general methods from the parent Missions class.  The high-level methods for the missions should be implemented in these classes.

The simulator is initiated by running `roslaunch usv16 simulator.launch`.

## Getting Started ##

* This ROS package should be usable on a Linux machine with a C++11 (or greater) compiler.
* This package was developed for ROS Indigo on Ubuntu 14.04; using other versions of ROS or Ubuntu may be problematic.
* For the simulator to compile properly, the user must edit lines 43 and 78 of the file 'rospackageswamv16/usv16_simulator/src/path_planner_server.cpp' and 66, 68, and 72 of the file 'rospackageswamv16/usv16_simulator/src/path_planner/TrajectoryPlanner.cpp' to the users local path
* To use this package, simply download it into `(your catkin workspace)/src` make the above changes for the simulator, and run `catkin_make`.

* For the simulator to work properly, once it is launched the user will need to select 'Add' from the bottom left hand
section of the RVIZ gui and add an 'AerialMapDisplay' object.  The user must then copy and paste: 'http://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}.jpg?access_token=pk.eyJ1IjoibGl0aGVyIiwiYSI6ImNpdWV3NjFwazAwY2wyb3J1eXZqMDJsN3cifQ.Dm8Zx4UU-H2nlINV6D-b8Q'
into the 'Object URI' box, overwriting whatever text is there.

##Dependencies##

ROS bare bones packages - installed during initial install.

ROS-Qt - can be installed using `sudo apt-get install ros-indigo-rqt*` if not installed during initial setup.

##TODO##
* Fully address build issue
* Make the GUI load with the correct perspective at launch
* Addess the insane path issues with usv16_simulator
* Add teleoperation
* Update the backend of the GUI
 - verify publish/subscribe channels
 - provide functionality to `Control Mode` options
 - Duplicate for `USV2` and `USV3`
* Update the interface of the GUI
 - More meaningful names/buttons for generating path
 - more streamlined layout

##NOTES ON THE GUI##
* There is an inital build issue having to do with message generation in the simulator packages.  The best solution at this time is to clear the src folder in the catkin workspace with the exception of the package usv16_simulator, run 'catkin_make' from the appropriate directory, add all additional folders back, and run catkin_make again.
* The GUI has been updated to the most current persepctive.  This can be accessed by running `roslaunch usv16 simulator.launch`.  Once the gui opens, on the rviz submenu select `file->open config-><path_to_cws/src/rospackageswamv16/usv16_simulator/config/wamv16.rviz`
* To generate a new path, select `2D Nav Goal`, click on the map where you would like the setpoint to be, and then drag in the direction of the vessel's desired final heading prior to letting go of the left mouse button.  Click apply on the bottom right corner to generate path.
* To add obstacles, select `Publish Point` and click on the map where you would like the object.  Objects placed in the path of the vehicle will result in an update path.
* The current path can be cancelled by selecting `Publish Point` and placing the marker on the location of the `2D Nav Goal`.
