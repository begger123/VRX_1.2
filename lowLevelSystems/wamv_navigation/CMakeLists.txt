cmake_minimum_required(VERSION 2.8.3)
project(wamv_navigation)

# crap out on c++11 check fail
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find_package(catkin REQUIRED roslaunch)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roslaunch
    std_msgs
    nav_msgs
    move_base_msgs
    geometry_msgs
    tf
    custom_messages_biggie
)

add_service_files(
    FILES
    SendGoal.srv
    CircleTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  custom_messages_biggie
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs nav_msgs geometry_msgs message_generation
)

include_directories(
  include
    ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_executable(circular_path src/circle_me.cpp)
target_link_libraries(circular_path ${catkin_LIBRARIES})
add_dependencies(circular_path wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(create_odom src/odom_to_base.cpp)
target_link_libraries(create_odom ${catkin_LIBRARIES})
add_dependencies(create_odom wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_to_ned src/path_to_ned.cpp)
target_link_libraries(path_to_ned ${catkin_LIBRARIES})
add_dependencies(path_to_ned wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_to_goal src/path_to_goal.cpp)
target_link_libraries(path_to_goal ${catkin_LIBRARIES})
add_dependencies(path_to_goal wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multiple_goals src/multiple_goals.cpp)
target_link_libraries(multiple_goals ${catkin_LIBRARIES})
add_dependencies(multiple_goals wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(send_array src/sendArrayGoals.cpp)
target_link_libraries(send_array ${catkin_LIBRARIES})
add_dependencies(send_array wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(send_single_goal src/sendSingleGoal.cpp)
target_link_libraries(send_single_goal ${catkin_LIBRARIES})
add_dependencies(send_single_goal wamv_navigation_generate_messages_cpp ${${custom_messages_biggie}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# install(
#   DIRECTORY config launch maps
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

