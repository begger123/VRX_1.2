#include <the_planner/Missions.h>

///////////////////////////////////Mission///////////////////////////////////
Mission::Mission(ros::NodeHandle &nh) : mission_nh_(&nh)
{
  //subscribes to state for various maneuvers
  state_subscriber = mission_nh_->subscribe("/p3d_wamv_ned", 10, &Mission::state_callback, this);
  //checks to see if waypoint is complete
  mission_subscriber = mission_nh_->subscribe("missionComplete", 10, &Mission::mission_callback, this);
  //directly to high_to_low_node
  custom_waypoint_array_publisher = mission_nh_->advertise<custom_messages_biggie::waypoint_array>("/waypoint_array", 10);
  //running through Armando's obstacle avoidance node
  pose_array_pub = mission_nh_->advertise<geometry_msgs::PoseArray>("array_goals", 1, true);

  this->client_goal     = mission_nh_->serviceClient<wamv_navigation::SendGoal>("send_goal");
  this->client_circle   = mission_nh_->serviceClient<wamv_navigation::CircleTarget>("circle_target");
  this->goal_status_sub = mission_nh_->subscribe("/move_base/result", 1, &Mission::goalStatusCallback, this);
  this->path_ned_pub    = mission_nh_->advertise<custom_messages_biggie::waypoint_array>("waypoint_array", 50);

  finished = false;
}

Mission::Mission()
{
  finished = false;
}

Mission::~Mission()
{

}

void Mission::loop()
{

}


void Mission::state_callback(const nav_msgs::Odometry::ConstPtr& Msg)
{
  theOdom.header = Msg->header;
  theOdom.child_frame_id = Msg->child_frame_id;
  theOdom.pose = Msg->pose;
  theOdom.twist = Msg->twist;
}

void Mission::mission_callback(const std_msgs::Bool::ConstPtr& Msg)
{
  missionComplete.data=Msg->data;
}

bool Mission::is_finished()
{
  return finished;
}

void Mission::goalStatusCallback(move_base_msgs::MoveBaseActionResult msg)
{
    ROS_DEBUG("previous_status = %d", previous_status);
    ROS_DEBUG("Goal Status = %u", msg.status.status);
    if (msg.status.status == 3) {
        goal_reached = true;
    }
    else if (msg.status.status == 2) {
        ROS_DEBUG("Status = %u", msg.status.status);
        goal_reached = false;
        previous_status = 2;
    }
}

////////////////////////////////////////////////////////////////////////////////////
/////////////This is the beginning of the motion primitive functions////////////////
////////////////////////////////////////////////////////////////////////////////////

//This function will place the x, y, and psi parameters into a station keeping mission
//This mission will then be published for the timeval that is set
/*void Mission::move_to_sk (float x, float y, float psi, double timeval)
{
  ROS_INFO("Inside of move_to_sk");
  double counter = ros::Time::now().toSec();
  missionTarget.data = {x, y, psi};
  missionTarget.type = 1;
  while((ros::Time::now().toSec()-counter)<timeval&&ros::ok())
  {
//    ROS_INFO((ros::Time::now().toSec()));
    this->publish_to_controller_NONBLOCK(missionTarget);
  }
  return;
}

//this function is used to publish a waypoint and velocity to the controller
void Mission::move_to (float x, float y, float vel)
{
  ROS_INFO("Inside move_to");
  missionTarget.data = {x, y, vel};
  missionTarget.type = 0;
  this->publish_to_controller(missionTarget);
  return;
}

//This function will return an x, y, or psi 
float Mission::get_curr (uint8_t which)
{
  ros::spinOnce();
  switch(which)
  {
    case X:
      //ROS_INFO("x is %f",theState.pose.x);
      return theState.pose.x;  
    break;
    case Y:
      //ROS_INFO("y is %f",theState.pose.y);
      return theState.pose.y;
    break;
    case PSI:
      //ROS_INFO("theta is %f",theState.pose.theta);
      return theState.pose.theta;
    break;
    default:
      //ROS_INFO("bad request in get_curr(), try again.");
    break;
  }
}

void Mission::set_head (float psi, double timeval)
{
  ROS_INFO("Inside set_head");
  float currx, curry;

  currx = get_curr(X);
  curry = get_curr(Y);
  usv16_ctrl::Usv16Reference referenceTarget;
  referenceTarget.t=ros::Time::now().toSec();
  referenceTarget.ref = {psi, currx, curry};
  referenceTarget.type = 0;
  double counter = ros::Time::now().toSec();
  ROS_INFO("Setting x,  y, and  psi to : %f,  %f, %f",currx, curry, psi);
  //3 degrees = 0.05 rad
  while(!sqrt((pow(get_curr(PSI)-psi,2))<0.05)&&ros::ok())
  {
  //  ROS_INFO("%f", ros::Time::now().toSec());
    //ROS_INFO("Setting x,  y, and  psi to : %f,  %f, %f",currx, curry, psi);
    this->publish_reference(referenceTarget);
    ros::Duration(.2).sleep();
  }
  return;
}

//Moves by some amount of meters using trajectory tracking, sets heading, holds that spot for a set timeval
void Mission::move_by (float delx, float dely, float vel, float delphi, float timeval)
{
  ROS_INFO("Inside move_by");
  float currx, curry, currpsi, x, y, psi;

  currx = get_curr(X);
  curry = get_curr(Y);
  currpsi = get_curr(PSI);
  x = currx + delx;
  y = curry + dely;
  psi = currpsi + delphi;
  //ROS_INFO("%f", currx);
  //ROS_INFO("%f", curry);
  //ROS_INFO("%f", currpsi);
  //ROS_INFO("%f", x);
  //ROS_INFO("%f", y);
  move_to(x, y, vel);
  set_head(psi,timeval);
  return;
}

void Mission::move_by_heading(float distance)
{
  ROS_INFO("Inside move_by_heading");
//  ROS_INFO("The north change is: %f", distance*cos(get_curr(PHI)));
//  ROS_INFO("The east change is: %f", distance*sin(get_curr(PHI)));
//  ROS_INFO("The heading change is: %f", get_curr(PHI));

  move_by(distance*cos(get_curr(PSI)), distance*sin(get_curr(PSI)), 1.5, get_curr(PSI),  10);
}*/




