#ifndef TOTEMS_H
#define TOTEMS_H

#include <the_planner/highLevelPlanner.h>
#include <the_planner/Missions.h>
#include <geometry_msgs/Point32.h>
#include <wamv_navigation/SendGoal.h>
#include <wamv_navigation/CircleTarget.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>


/* Mission Description: The task requires the vehicle to locate and completely circle
 *     two distinct objects (totems). For practice and qualifying days, the totems may
 *     be placed within a field of obstacles. The totem area may be marked by four white
 *     buoys around an area of 40x40 meters.
 *
 * Input data: The position and color of the totems, the order of the sequence for the
 *     vehicle to traverse them, and the direction the totems must be circled.
 *
 * Mission sequence:
 *     1. Get totem_order sequence for the vehicle to traverse.
 *     2. Get direction of circling for each of the two totems.
 *     3. Get position for both totems (from perception system).
 *     4. Approach first totem in the sequence at a predefined distance to start circling operation.
 *     5. Execute circling behavior (function) with the following arguments:
 *             - Current vehicle's position.
 *             - Direction of circling.
 *             - Radius of circle.
 *     6. Once it finishes circling the first totem, search for the position of the second one,
 *        if not already available.
 *     7. Repeat steps 4-6 for the second totem.
 *     8. Move to the next task.  End of mission. */
       
struct TotemStruct {
    geometry_msgs::Point32 totem_pos;
    int color;
    int order;
    bool clockwise;
    bool pos_detected;
    bool color_detected;
};

// struct TotemsPos {


class totems : public Mission
{
private:
    ros::Subscriber totems_colors_sub;
    ros::Subscriber totems_dir_sub;
    ros::Subscriber totems_pos_sub;
    ros::Subscriber totems_sub;
    // geometry_msgs::Point32 totem1_pos;
    // geometry_msgs::Point32 totem2_pos;
    double Rc;                              // Circle radius
    double Rs;                              // Radius to start circling operation
    double outer_radius;
    double delta_radius_circle;
    double radius_circle;
    double vehicle_pos[2];
    double totem1_pos[2];
    double totem2_pos[2];
    double goal_start[2];
    double goal_finish[2];
    double goal_tol;
    int current_status;
    int totems_order[3];
    int sorted_seq[3];
    int totem_count;
    int totem1_idx;
    int totem2_idx;
    int idx;
    int totem_idx[3];
    bool totem_found[3];
    bool totem1_found;
    bool totem2_found;
    bool clock_rotation;
    bool first_totem1;
    bool clockwise[2];
    void goalStatusCallback(move_base_msgs::MoveBaseActionResult msg);
    struct TotemStruct totems_arr[3];
    std::vector<std::vector<double> > totems_pos_arr;

    // The information should be introduced in the predefined navigation order 
    // std::vector<std::string> totems_col;
    // std::vector<std::string> totems_dir;
    // std::vector<std::vector<float> > totems_pos;

public:
    totems(ros::NodeHandle &nh);
    ~totems();
    void mission_callback(const std_msgs::String::ConstPtr& msg);
    void totemStuffCallback(visualization_msgs::MarkerArray msg);
    void vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg);
    void loop();
    enum Sequence{START, SEARCH_TOTEMS_1, GOTO_TOTEM_1, CIRCLE_TOTEM_1, SEARCH_TOTEMS_2, GOTO_TOTEM_2, CIRCLE_TOTEM_2, FINISH};
    // enum Sequence{START, GOTO_TOTEM_1, CIRCLE_TOTEM_1, GOTO_TOTEM_2, CIRCLE_TOTEM_2, FINISH};
    enum TotemColor{ROJO, VERDE, AZUL};
};

#endif /* TOTEMS_MISSION_H */
