cmake_minimum_required(VERSION 2.8.3)
project(sulu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  dynamixel_controllers
  laser_assembler
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
  image_geometry
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   vision.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision src/experiment.cpp)
target_link_libraries(vision opencv_core opencv_imgproc opencv_highgui opencv_calib3d opencv_contrib opencv_features2d opencv_flann opencv_gpu opencv_legacy opencv_ml opencv_objdetect opencv_photo opencv_stitching opencv_superres opencv_video opencv_videostab ${catkin_LIBRARIES})
add_dependencies(vision sulu_generate_messages_cpp)

add_executable(tiltNnodder src/tiltNnodder.cpp)
target_link_libraries(tiltNnodder ${catkin_LIBRARIES})
add_dependencies(tiltNnodder sulu_gencpp)

add_executable(lidarFramePublisher src/lidarFramePublisher.cpp)
target_link_libraries(lidarFramePublisher ${catkin_LIBRARIES})
add_dependencies(lidarFramePublisher sulu_gencpp)
