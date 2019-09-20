#ifndef ACOUSTICS_CHANNEL_H
#define ACOUSTICS_CHANNEL_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>
#include <geometry_msgs/Point32.h>
#include <wamv_navigation/SendGoal.h>
#include <wamv_navigation/CircleTarget.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>


class acoustics_channel : public Mission
{
private:
    double radius_circle;
    double delta_radius_circle;
    double marker_pos[2];
    double vehicle_pos[2];
    void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
    double go_initial[2];
    double go_midpoint[2];
    double go_intermed_in[2];
    double go_marker[2];
    double ret_intermed_in[2];
    double ret_midpoint[2];
    double ret_finish[2];
    double ret_home[2];
    
public:
    acoustics_channel(ros::NodeHandle &nh);
    ~acoustics_channel();
    void vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg);
    void loop();
    enum Sequence{START, GOTO_MIDPOINT, GOTO_INTERMEDIATE_IN, GOTO_MARKER, CIRCLE_MARKER, RETURN_INTERMEDIATE_IN, RETURN_MIDPOINT, FINISH};
};

#endif /* ACOUSTICS_CHANNEL_H */

