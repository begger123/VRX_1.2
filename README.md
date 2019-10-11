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
