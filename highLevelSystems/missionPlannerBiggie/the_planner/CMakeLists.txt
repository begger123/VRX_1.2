cmake_minimum_required(VERSION 2.8.3)
project(the_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf
)

# Non-ROS dependencies
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

# crap out on c++11 check fail
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   mission_out.msg
   waypoint_list.msg
   line.msg
   selectedCSP.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   initTraj.srv
 )

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy geometry_msgs message_generation std_msgs nav_msgs
  DEPENDS Eigen
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

find_package(the_planner REQUIRED)
include_directories(${the_planner_INCLUDE_DIRS})

########################
## Planner Executable ##
########################

set(MISSIONS src/missions/tester.cpp src/missions/navigation_channel.cpp src/missions/acoustics_channel.cpp src/missions/buoy_field.cpp src/missions/totems.cpp)

#set(MISSIONS src/missions/tester.cpp src/missions/listen_to_gui.cpp src/missions/slalom_mission.cpp src/missions/speed_gate_mission.cpp  src/missions/follow_the_leader.cpp)

add_executable(planner src/plannerV3.cpp src/highLevelPlanner.cpp src/Missions.cpp ${MISSIONS})
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner the_planner_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#######################
## Trajectory Server ##
#######################

add_executable(gen_traj_server src/initialTrajectory/initial_trajectory.cpp src/initialTrajectory/init_traj_server.cpp)
target_link_libraries(gen_traj_server ${catkin_LIBRARIES})
add_dependencies(gen_traj_server the_planner_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Small Persistance table simulation to try Totems Mission
add_executable(fake_persist_table src/missions/sim_persist_table)
target_link_libraries(fake_persist_table ${catkin_LIBRARIES})

