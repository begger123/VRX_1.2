cmake_minimum_required(VERSION 2.8.3)
project(shape_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

# crap out on c++11 check fail
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   mission_out.msg
# )

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
#  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# include_directories()
include_directories( include
  ${catkin_INCLUDE_DIRS}
)

find_package(shape_detector REQUIRED)
include_directories(${shape_detector_INCLUDE_DIRS})


set(MISSIONS src/missions/tester.cpp src/missions/listen_to_gui.cpp)

add_executable(shape_detector_node src/shape_detector_node.cpp)
target_link_libraries(shape_detector_node ${catkin_LIBRARIES})
add_dependencies(shape_detector_node shape_detector_generate_messages_cpp)
