#ifndef DOCK_H
#define DOCK_H

#include <iostream>
#include <vector>
#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>
#include <geometry_msgs/Point32.h>
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
    std::vector<double> dock_path;
    std::vector<double> dock_path1;
    std::vector<double> dock_path2;

    bool got_around_points = false;
    bool got_path_points = false;
    bool got_etat = false;
    bool got_plackard = false;
    bool isPlackard = true;
    bool got_entire_dock = false;
    bool got_part_dock = false;

    int size_around = 8;
    int size_path = 8;

    // Station-keeping goal tolerances
    float tol_x = 1.0;    // in meters
    float tol_y = 1.0;    // in meters
    float tol_psi = 10.0;  // in degrees

    double sk_heading = 0;
    double sk_heading1 = 0;
    double sk_heading2 = 0;
    double sk1_heading = 0;
    double sk2_heading = 0;
    double sk3_heading = 0;
    double x = 0;
    double y = 0;
    double yaw_angle = 0;
    double dock_around[8];
    double goal_start[2];
    double sk_etat[3];

    // ROS stuff...
    ros::Subscriber plackard_sub;
    ros::Subscriber dockpath_sub;
    ros::Subscriber dockaround_sub;
    ros::Subscriber pose_sub;
    ros::Publisher stationkeep_pub;
    ros::Time prev_time;
    ros::Duration delta_t;
    geometry_msgs::Pose2D skPoint;

    // Callback functions
    void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
    void dockPathCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void dockAroundCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void plackardCallback(const std_msgs::BoolConstPtr &msg);
    void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg);

public:
    dock(ros::NodeHandle &nh);
    ~dock();
    void loop();
    enum Sequence{START, EXPLORE_DOCK, DOCKPATH1_START, DOCKPATH1_SK1, DOCKPATH1_STOP, DOCKPATH1_SK2, DOCKPATH1_SK3, DOCKPATH1_SK1_REV,
                                       DOCKPATH2_START, DOCKPATH2_SK1, DOCKPATH2_STOP, DOCKPATH2_SK2, DOCKPATH2_SK3, DOCKPATH2_SK1_REV,   
                                       CIRCLE_SEARCH, GOTO_STATION2, MISSION_FINISHED};

    // Some functions
    double twopiwrap(double angle);
    double piwrap(double angle);
};

#endif /* DOCK_H */


