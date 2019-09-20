#include <ros/ros.h>
#include <wamv_navigation/CircleTarget.h>
// #include <custom_messages_biggie/waypoint_array.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <cmath>


/* This function gets information regarding the circle-radius, direction of travel, 
 * vehicle-pos and target-pos, and returns the circular path in the desired 
 * sequence and direction.
 * All the computations are made according to the ENU convention.*/

// Sorting function for vector of vectors, with respect to the first column of each vector
bool sortcol(const std::vector<double> &v1, const std::vector<double> &v2)
{
    return v1[0] < v2[0];
}


class CircleMe 
{
    private:
        ros::NodeHandle node;
        ros::Publisher circle_enu_pub;
        ros::ServiceServer circle_me_srv;
        double radius;
        bool clockwise;
        int num_wp;
        double arc_len_th;
        double start_heading_th;
        double vehicle_pos[2];
        double target_pos[2];

    public:
        CircleMe()
        {
            ROS_INFO("CircleTarget service has initiated...");
            // Some default values
            radius = 0.0;
            clockwise = false;
            num_wp = 9;                     // This corresponds to dividing each quadrant into two region
            arc_len_th = 3.0;               // 3 meters sounds good
            start_heading_th = M_PI/6;      // 30 deg. is the threshold

            // vehicle_pos and target_pos vectors will be filled out 
            // in the corresponding subscriber callback functions
            vehicle_pos[0] = 0.0;
            vehicle_pos[1] = 0.0;
            target_pos[0]  = 0.0;
            target_pos[1]  = 0.0;

            // ROS stuff...
            // state_ned_sub  = node.subscribe("/p3d_wamv_ned", 10, &CircleMe::vehiclePosCallback, this);
            // first_totem_sub = node.subscribe("/target_pos", 10, &CircleMe::targetPosCallback, this);
            circle_enu_pub = node.advertise<geometry_msgs::PoseArray>("waypoint_array", 10);
            circle_me_srv  = node.advertiseService<wamv_navigation::CircleTarget::Request, wamv_navigation::CircleTarget::Response>("circle_target",
                                                                                           boost::bind(&CircleMe::doCircleCallback, this, _1, _2));
        }

    private:
        bool doCircleCallback(wamv_navigation::CircleTarget::Request &req, wamv_navigation::CircleTarget::Response &res)
        {
            // First store service-variables
            vehicle_pos[0] = req.vehicle_pos.y;
            vehicle_pos[1] = req.vehicle_pos.x;
            target_pos[0]  = req.target_pos.y;
            target_pos[1]  = req.target_pos.x;
            radius         = req.radius;
            clockwise      = req.clockwise;                 // Remember clockwise is negative on ENU

            // Find the number of waypoints over the circular 
            // path w.r.t a reasonable value of arc_length
            std::vector<std::vector<double> > waypoints;
            double arc_len = 2*M_PI*radius/(num_wp - 1);
            ROS_DEBUG("arc length = %g", arc_len);
            while (arc_len < arc_len_th) {
                num_wp = num_wp - 1;
                arc_len = 2*M_PI*radius/(num_wp - 1);
            }
            ROS_DEBUG("arc length = %g", arc_len);
            ROS_DEBUG("radius = %g", radius);
            ROS_DEBUG("Number of waypoints = %d", num_wp);

            // Now let's fill out the list of waypoints
            double theta = 0.0;                             // This is the first angle value corresponding to first wp
            ROS_INFO("target_pos = [%g, %g]", target_pos[0], target_pos[1]);
            for (int i = 0; i < num_wp; i++) {
                std::vector<double> wp;
                wp.push_back(radius*cos(theta) + target_pos[0]);
                wp.push_back(radius*sin(theta) + target_pos[1]);
                waypoints.push_back(wp);
                // ROS_DEBUG("wp = [%g, %g]", wp[0], wp[1]);
                // ROS_DEBUG("theta = %g", theta*180/M_PI);
                theta += arc_len/radius;
            }
            printf("waypoints = [\n");
            for (unsigned long i = 0; i < waypoints.size(); i++) {
                printf("%g, %g,\n", waypoints[i][0], waypoints[i][1]);
            }
            printf("]\n");

            // Compute the two closest waypoints
            std::vector<std::vector<double> > dist_wps;
            double dist = 0.0;
            for (int i = 0; i < num_wp; i++) {
                std::vector<double> one_elem;
                dist = sqrt(pow(waypoints[i][0] - vehicle_pos[0], 2) + pow(waypoints[i][1] - vehicle_pos[1], 2));
                one_elem.push_back(dist);
                one_elem.push_back(i);
                dist_wps.push_back(one_elem);
            }
            printf("dist_wps = [\n");
            for (unsigned long i = 0; i < waypoints.size(); i++) {
                printf("%g, %g\n", dist_wps[i][0], dist_wps[i][1]);
            }
            printf("]\n");

            // std::vector<std::vector<double> > dist_wps_sort;
            std::sort(dist_wps.begin(), dist_wps.end(), sortcol);
            printf("dist_wps_sort = [\n");
            for (unsigned long i = 0; i < waypoints.size(); i++) {
                printf("%g, %g\n", dist_wps[i][0], dist_wps[i][1]);
            }
            printf("]\n");

            double smallest[3]        = {waypoints[dist_wps[0][1]][0], waypoints[dist_wps[0][1]][1], dist_wps[0][1]};
            double second_smallest[3] = {waypoints[dist_wps[1][1]][0], waypoints[dist_wps[1][1]][1], dist_wps[1][1]};
            ROS_DEBUG("The two closest waypoints to the vehicle:");
            ROS_DEBUG("smallest = [%g, %g]", smallest[0], smallest[1]);
            ROS_DEBUG("second_smallest = [%g, %g]", second_smallest[0], second_smallest[1]);
            ROS_DEBUG("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            // Compute the first waypoint according to the direction of travel.
            // It returns the waypoint index and start_heading
            bool out = true;
            double *first_wp = choose_first_waypoint(smallest, second_smallest, out);
            int idx_first_wp = (int) first_wp[0];
            double start_heading = first_wp[1];
            delete[] first_wp;
            ROS_DEBUG("start_heading_before = %g", start_heading);

            // Check if starting heading is larger than min threshold.
            // If not move to an adjacent waypoint in the sequence according 
            // to the direction of circling
            if (start_heading < start_heading_th) {
                if (clockwise) {
                    idx_first_wp--;
                    ROS_DEBUG("We'd decreased the index due to a small start_heading in CCW circling");
                }
                else {
                    idx_first_wp++;
                    ROS_DEBUG("We'd increased the index due to a small start_heading in CW circling");
                }
            }

            // Redefine the list of waypoints according to the new criteria based on
            // closest waypoint from vehicle and direction of circling (cloclwise or counter-cw)
            int j, k = 0;
            // std::vector<std::vector<double> > circular_path;
            geometry_msgs::Pose elem;
            geometry_msgs::PoseArray circular_path;
            if (clockwise) {
                for (unsigned long i = 0; i < waypoints.size(); i++) {
                    j = (idx_first_wp - i) % 8;
                    // circular_path.push_back(waypoints[j]);
                    elem.position.x = waypoints[j][0];
                    elem.position.y = waypoints[j][1];
                    elem.orientation.z = 0.0;
                    elem.orientation.w = 1.0;
                    circular_path.poses.push_back(elem);
                }
            }
            else {
                for (unsigned long i = 0; i < waypoints.size(); i++) {
                    k = (idx_first_wp + i) % 8;
                    // circular_path.push_back(waypoints[k]);
                    elem.position.x    = waypoints[k][0];
                    elem.position.y    = waypoints[k][1];
                    elem.orientation.z = 0.0;
                    elem.orientation.w = 1.0;
                    circular_path.poses.push_back(elem);
                }
            }
            printf("circular_path = [\n");
            for (unsigned long i = 0; i < waypoints.size(); i++) {
                printf("%g, %g\n", circular_path.poses[i].position.x, circular_path.poses[i].position.y);
            }
            printf("]\n");

            // Now let's convert the circular path to NED in order to properly send
            // it to the path-planner in path_to_goal.cpp
            geometry_msgs::Pose el;
            geometry_msgs::PoseArray circular_ned;
            for ( unsigned long i = 0; i < waypoints.size(); i++) {
                el.position.x = circular_path.poses[i].position.y;
                el.position.y = circular_path.poses[i].position.x;
                circular_ned.poses.push_back(el);
            }

            res.circle_path = circular_ned;
            return true;
        }

    public:
        double* choose_first_waypoint(double smallest[3], double sec_smallest[3], bool out)
        {
            /* This function choose as the first waypoint the one that is not only closer but
             * that complies with the correct direction of navigation across the circular path.
             * It returns the waypoint index, from the original waypoints array, that corresponds
             * to the choosen first waypoint */
            double vect_target[2]     = {target_pos[0] - vehicle_pos[0], target_pos[1] - vehicle_pos[1]};
            double vect_smallest[2]   = {smallest[0] - vehicle_pos[0], smallest[1] - vehicle_pos[1]};
            double vect_2smallest[2]  = {sec_smallest[0] - vehicle_pos[0], sec_smallest[1] - vehicle_pos[1]};
            double mag_vect_target    = sqrt(pow(vect_target[0], 2) + pow(vect_target[1], 2));
            double mag_vect_smallest  = sqrt(pow(vect_smallest[0], 2) + pow(vect_smallest[1], 2));
            double mag_vect_2smallest = sqrt(pow(vect_2smallest[0], 2) + pow(vect_2smallest[1], 2));

            // Compute scalar rejection (normal orthogonal projection) between the vectors
            // Also compute the start_heading over the circular path for the choosen waypoint.
            double reject_smallest  = (vect_smallest[1]*vect_target[0] - vect_smallest[0]*vect_target[1]) / mag_vect_target;
            double reject_2smallest = (vect_2smallest[1]*vect_target[0] - vect_2smallest[0]*vect_target[1]) / mag_vect_target;
            double start_heading = 0.0;
            double *ret = new double[2];

            ROS_INFO("smallest = [%g,%g]; sec_smallest = [%g,%g]", smallest[0], smallest[1], sec_smallest[0], sec_smallest[1]);
            ROS_INFO("vect_smallest = [%g,%g]; vect_2smallest = [%g,%g]", vect_smallest[0], vect_smallest[1], vect_2smallest[0], vect_2smallest[1]);
            ROS_INFO("reject_smallest = %g, reject_2smallest = %g", reject_smallest, reject_2smallest);

            if (clockwise == true) {
                if (reject_smallest > 0) {
                    double num = vect_smallest[0]*vect_target[0] + vect_smallest[1]*vect_target[1];
                    double den = mag_vect_smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) smallest[2];      // This corresponds to the index
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the smallest-clockwise = [%g, %g], index = %d", smallest[0], smallest[1], (int) smallest[2]);
                    return ret;
                }
                else if (reject_2smallest > 0) {
                    double num = vect_2smallest[0]*vect_target[0] + vect_2smallest[1]*vect_target[1];
                    double den = mag_vect_2smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) sec_smallest[2];
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the second-smallest-clockwise = [%g, %g], index = %d", sec_smallest[0], sec_smallest[1], (int) sec_smallest[2]);
                    return ret;
                }
                else {
                    ROS_ERROR("Something went wrong computing the rejection component");
                }
            }
            else {
                if (reject_smallest < 0) {
                    double num = vect_smallest[0]*vect_target[0] + vect_smallest[1]*vect_target[1];
                    double den = mag_vect_smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) smallest[2];
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the smallest-counter-cw = [%g, %g], index = %d", smallest[0], smallest[1], (int) smallest[2]);
                    return ret;
                }
                else if (reject_2smallest < 0) {
                    double num = vect_2smallest[0]*vect_target[0] + vect_2smallest[1]*vect_target[1];
                    double den = mag_vect_2smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) sec_smallest[2];
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the second-smallest-counter-cw = [%g, %g], index = %d", sec_smallest[0], sec_smallest[1], (int) sec_smallest[2]);
                    return ret;
                }
                else {
                    ROS_ERROR("Something went wrong computing the rejection component");
                }
            }
        }
        
        double* choose_first_waypoint_in(double smallest[3], double sec_smallest[3])
        {
            /* This function choose as the first waypoint the one that is not only closer but
             * that complies with the correct direction of navigation across the circular path.
             * It returns the waypoint index, from the original waypoints array, that corresponds
             * to the choosen first waypoint */
            double vect_target[2]     = {target_pos[0] - vehicle_pos[0], target_pos[1] - vehicle_pos[1]};
            double vect_smallest[2]   = {smallest[0] - vehicle_pos[0], smallest[1] - vehicle_pos[1]};
            double vect_2smallest[2]  = {sec_smallest[0] - vehicle_pos[0], sec_smallest[1] - vehicle_pos[1]};
            double mag_vect_target    = sqrt(pow(vect_target[0], 2) + pow(vect_target[1], 2));
            double mag_vect_smallest  = sqrt(pow(vect_smallest[0], 2) + pow(vect_smallest[1], 2));
            double mag_vect_2smallest = sqrt(pow(vect_2smallest[0], 2) + pow(vect_2smallest[1], 2));

            // Compute scalar rejection (normal orthogonal projection) between the vectors
            // Also compute the start_heading over the circular path for the choosen waypoint.
            double reject_smallest  = (vect_smallest[1]*vect_target[0] - vect_smallest[0]*vect_target[1]) / mag_vect_target;
            double reject_2smallest = (vect_2smallest[1]*vect_target[0] - vect_2smallest[0]*vect_target[1]) / mag_vect_target;
            double start_heading = 0.0;
            double *ret = new double[2];

            ROS_INFO("smallest = [%g,%g]; sec_smallest = [%g,%g]", smallest[0], smallest[1], sec_smallest[0], sec_smallest[1]);
            ROS_INFO("vect_smallest = [%g,%g]; vect_2smallest = [%g,%g]", vect_smallest[0], vect_smallest[1], vect_2smallest[0], vect_2smallest[1]);
            ROS_INFO("reject_smallest = %g, reject_2smallest = %g", reject_smallest, reject_2smallest);

            if (clockwise == true) {
                if (reject_smallest > 0) {
                    double num = vect_smallest[0]*vect_target[0] + vect_smallest[1]*vect_target[1];
                    double den = mag_vect_smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) smallest[2];      // This corresponds to the index
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the smallest-clockwise = [%g, %g], index = %d", smallest[0], smallest[1], (int) smallest[2]);
                    return ret;
                }
                else if (reject_2smallest > 0) {
                    double num = vect_2smallest[0]*vect_target[0] + vect_2smallest[1]*vect_target[1];
                    double den = mag_vect_2smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) sec_smallest[2];
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the second-smallest-clockwise = [%g, %g], index = %d", sec_smallest[0], sec_smallest[1], (int) sec_smallest[2]);
                    return ret;
                }
                else {
                    ROS_ERROR("Something went wrong computing the rejection component");
                }
            }
            else {
                if (reject_smallest < 0) {
                    double num = vect_smallest[0]*vect_target[0] + vect_smallest[1]*vect_target[1];
                    double den = mag_vect_smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) smallest[2];
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the smallest-counter-cw = [%g, %g], index = %d", smallest[0], smallest[1], (int) smallest[2]);
                    return ret;
                }
                else if (reject_2smallest < 0) {
                    double num = vect_2smallest[0]*vect_target[0] + vect_2smallest[1]*vect_target[1];
                    double den = mag_vect_2smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) sec_smallest[2];
                    ret[1] = start_heading;
                    ROS_INFO("We've picked the second-smallest-counter-cw = [%g, %g], index = %d", sec_smallest[0], sec_smallest[1], (int) sec_smallest[2]);
                    return ret;
                }
                else {
                    ROS_ERROR("Something went wrong computing the rejection component");
                }
            }
        }
        // void targetPosCallback(geometry_msgs::Point32::ConstPtr & msg)
        // {
        //     // Fill out the
        // void vehiclePosCallback(const nav_msgs::OdometryConstPtr &msg)
        // {
        //     // Fill out vehicle's NED position (don't need the heading)
        //     vehicle_pos[0] = msg->pose.pose.position.x;
        //     vehicle_pos[1] = msg->pose.pose.position.y;
        // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_path");
    CircleMe circleMe;
    ros::spin();
    return 0;
}


                




