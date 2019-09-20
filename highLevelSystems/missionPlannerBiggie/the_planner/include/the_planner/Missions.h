#ifndef MISSIONS_H
#define MISSIONS_H
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include "custom_messages_biggie/waypoint_array.h"
#include "the_planner/initTraj.h"
#include <tf/transform_listener.h>
#include "the_planner/objects.h"
#include "the_planner/waypoint_list.h"
#include "usv_ahc_py/buoy.h"

#include "wamv_navigation/SendGoal.h"
#include "wamv_navigation/CircleTarget.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>

//Mission class used for inheritance
class Mission
{
 public:
  Mission(ros::NodeHandle &nh);
  Mission();
  ~Mission();
  void state_callback(const nav_msgs::Odometry::ConstPtr& Msg);
  void mission_callback(const std_msgs::Bool::ConstPtr& Msg);
  bool is_finished();
  void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
  virtual void loop();

  //THIS IS THE BEGINNING OF THE VILLANOVA DEVELOPED TRANSITING FUNCTIONS
  float get_curr (uint8_t which);
  void move_to_sk (float x, float y, float phi, double timeval);
  void move_to (float x, float y, float vel);
  void set_head (float phi, double timeval);
  void move_by (float delx, float dely, float vel, float delphi, float timeval);
  void move_by_heading(float distance);

  std::string mission_name;
  float timeout;
  float north;
  float east;
  float heading;
  int task;

 protected:
  double begin_time;
  double time_remaining;
  bool finished;
  int gateNumber;

  std_msgs::Bool missionComplete;
  nav_msgs::Odometry theOdom;

  ros::NodeHandle *mission_nh_;
  ros::Subscriber state_subscriber;
  ros::Subscriber mission_subscriber;
  ros::Publisher custom_waypoint_array_publisher;
  ros::Publisher pose_array_pub;
  tf::TransformListener lidar_nwu_to_ned_origin_tf;
  tf::StampedTransform the_tf;

  ros::ServiceClient client_goal;
  ros::ServiceClient client_circle;
  ros::Subscriber goal_status_sub;
  ros::Publisher  path_ned_pub;
  ros::Subscriber state_ned_sub;
  int previous_status;
  bool goal_reached;
};

#endif


