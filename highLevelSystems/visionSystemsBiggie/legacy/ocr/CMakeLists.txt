cmake_minimum_required(VERSION 2.8.3)
project(ocr)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
roscpp
message_generation
std_msgs
cv_bridge
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ocr.msg
 )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(ocr src/main.cpp)
target_link_libraries(ocr -llept -Lleptonica tesseract opencv_core opencv_imgproc opencv_highgui opencv_calib3d opencv_contrib opencv_features2d opencv_flann opencv_gpu opencv_legacy opencv_ml opencv_objdetect opencv_photo opencv_stitching opencv_superres opencv_video opencv_videostab ${catkin_LIBRARIES})
add_dependencies(ocr ocr_generate_messages_cpp)
