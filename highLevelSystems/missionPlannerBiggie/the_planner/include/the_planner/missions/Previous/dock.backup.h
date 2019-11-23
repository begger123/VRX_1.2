#ifndef DOCK_H
#define DOCK_H

#include <iostream>
#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>
#include <geometry_msgs/Point32.h>
// #include <geometry_msgs/Point2D.h>
#include <wamv_navigation/SendGoal.h>
#include <wamv_navigation/CircleTarget.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>


/* Mission Description: The vehicle starts searching for a small region involving the plackard, which is
 * what stands out from the dock from a far distance (it has more height). Once its sees it, it computs dock_explore
 * points and traverse them counter-clockwise in order to build a complete map of the dock. Then, from the dock map
 * it computes dock-aligned points describing a path suitable for the docking operation. Once it reaches the docking-point
 * it looks for the plackard information to see if it's the correct docking station.  If not, it computs the dock-aligned path
 * corresponding to the other docking station and goes there using the dock_explore points. Finally it moves through this path
 * until it arrives to the new docking-point where it'll station keep
 *
 * Input data: dock-explore-points, dock-path-points for each of the docking stations
 *
 * Mission sequence:
 *      1. 
 */


class dock : public Mission
{
private:
    bool got_explore_points;
    bool got_path_points;
    int size_explore;
    int size_path;
    double dock_explore[8];
    double dock_path[8];
    double vehicle_pos[2];
    double goal_start[2];
    double sk_heading = 0;

    // ROS stuff...
    ros::Subscriber plackard_sub;
    ros::Subscriber dockpath_sub;
    ros::Subscriber dockexplore_sub;
    ros::Publisher stationkeep_pub;
    geometry_msgs::Pose2D skPoint;
    void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
    void dockPathCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void dockExploreCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void plackardCallback(const std_msgs::BoolConstPtr &msg);

public:
    dock(ros::NodeHandle &nh);
    ~dock();
    void vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg);
    void loop();
    enum Sequence{START, MAP_DOCK, DOCKPATH1_START, DOCKPATH1_START_SK, DOCKPATH1_STOP, DOCKPATH1_SK1, DOCKPATH1_SK2, DOCKPATH1_SK_START};
};

#endif /* DOCK_H */
