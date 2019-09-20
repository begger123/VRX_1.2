#ifndef ACOUSTICS_CHANNEL2_H
#define ACOUSTICS_CHANNEL2_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>
#include <geometry_msgs/Point32.h>
#include <wamv_navigation/SendGoal.h>
#include <wamv_navigation/CircleTarget.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>


class acoustics_channel2 : public Mission
{
private:
    double radius_circle;
    double delta_radius_circle;
    double marker_pos[2];
    double vehicle_pos[2];
    void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
    
public:
    acoustics_channel2(ros::NodeHandle &nh);
    ~acoustics_channel2();
    void vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg);
    void loop();
    enum Sequence{START, GOTO_MIDPOINT, GOTO_INTERMEDIATE_IN, GOTO_MARKER, CIRCLE_MARKER, RETURN_INTERMEDIATE_IN, RETURN_MIDPOINT, FINISH};
};

#endif /* TOTEMS_MISSION_H */

