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
#include <vrx_gazebo/Task.h>


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

    bool got_explore_points = false;
    bool got_path_points = false;
    bool got_etat = false;
    bool got_placard = false;
    bool got_desiredPlacard = false;
    bool isPlacard = false;
    std::string thePlacardString;
    std::string theDesiredPlacardString;
    bool got_dock = false;

    int size_explore;
    int size_path;

    // Station-keeping goal tolerances
    float tol_x = 1.0;    // in meters
    float tol_y = 1.0;    // in meters
    float tol_psi = 10.0;  // in degrees

    double dock_explore[8];
    double x, y, yaw_angle;
    double goal_start[2];
    double sk_heading, sk_heading1, sk_heading2;
    double sk1_heading, sk2_heading, sk3_heading;
    double sk_etat[3];

    // ROS stuff...
    ros::Subscriber placard_sub;
    ros::Subscriber task_sub;
    ros::Subscriber dockpath_sub;
    ros::Subscriber dockexplore_sub;
    ros::Subscriber pose_sub;
    ros::Publisher stationkeep_pub;
    ros::Time prev_time;
    ros::Duration delta_t;
    geometry_msgs::Pose2D skPoint;

    // Callback functions
    void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
    void dockPathCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void dockExploreCallback(const std_msgs::Float32MultiArrayConstPtr &msg);
    void placardCallback(const std_msgs::StringConstPtr &msg);
    void desiredPlacardCallback(const std_msgs::StringConstPtr &msg);
    void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void checkPlacard();

public:
    dock(ros::NodeHandle &nh);
    ~dock();
    void loop();
    // enum Sequence{START, EXPLORE_DOCK, DOCKPATH1_START, DOCKPATH1_SK1, DOCKPATH1_STOP, DOCKPATH1_SK2, DOCKPATH1_SK3, DOCKPATH1_SK1_REV,
    //               EXPLORE_DOCK2, DOCKPATH2_START, DOCKPATH2_SK1, DOCKPATH2_STOP, DOCKPATH2_SK2};
    enum Sequence{START, EXPLORE_DOCK, DOCKPATH1_START, DOCKPATH1_SK1, DOCKPATH1_STOP, DOCKPATH1_SK2, DOCKPATH1_SK3, DOCKPATH1_SK1_REV, CIRCLE_SEARCH, GOTO_STATION2, MISSION_FINISHED};
                  // EXPLORE_DOCK2, DOCKPATH2_START};

    // Some functions
    double twopiwrap(double angle);
    double piwrap(double angle);
};

#endif /* DOCK_H */


