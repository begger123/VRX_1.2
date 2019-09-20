#include <ros/ros.h>
#include <wamv_navigation/CircleTarget.h>
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
        int return_wp;
        int M;
        double arc_len_th;
        double start_heading_th_out;
        double start_heading_th_in;
        double vehicle_pos[2];
        double target_pos[2];
        double delta_radius;

    public:
        CircleMe()
        {
            ROS_INFO("CircleTarget service has initiated...");
            // Some default values
            radius = 0.0;
            delta_radius = 2.0;
            clockwise = false;
            num_wp = 5;                     // This corresponds to dividing each quadrant into two region
            arc_len_th = 3.0;               // 3 meters sounds good
            start_heading_th_out = M_PI/6;      // 20 deg. is the threshold
            start_heading_th_in  = M_PI/4;

            // vehicle_pos and target_pos vectors will be filled out 
            // in the corresponding subscriber callback functions
            vehicle_pos[0] = 0.0;
            vehicle_pos[1] = 0.0;
            target_pos[0]  = 0.0;
            target_pos[1]  = 0.0;

            // ROS stuff...
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
            num_wp         = req.num_wp;
            return_wp      = req.return_wp;

            M = num_wp - 1;

            // Find the number of waypoints over the circular 
            // path w.r.t a reasonable value of arc_length
            std::vector<std::vector<double> > waypoints;
            double arc_len = 2*M_PI*radius/(num_wp - 1);
            ROS_INFO("arc length = %g", arc_len);
            while (arc_len < arc_len_th) {
                num_wp = num_wp - 1;
                arc_len = 2*M_PI*radius/(num_wp - 1);
            }
            ROS_INFO("arc length = %g", arc_len);
            ROS_INFO("radius = %g", radius);
            ROS_INFO("Number of waypoints = %d", num_wp);

            // Now let's fill out the list of waypoints
            double theta = 0.0;                             // This is the first angle value corresponding to first wp
            ROS_INFO("target_pos = [%g, %g]", target_pos[0], target_pos[1]);
            for (int i = 0; i < num_wp; i++) {
                std::vector<double> wp;
                wp.push_back(radius*cos(theta) + target_pos[0]);
                wp.push_back(radius*sin(theta) + target_pos[1]);
                waypoints.push_back(wp);
                // ROS_INFO("wp = [%g, %g]", wp[0], wp[1]);
                // ROS_INFO("theta = %g", theta*180/M_PI);
                theta += arc_len/radius;
            }
            printf("waypoints = [\n");
            for (unsigned long i = 0; i < waypoints.size(); i++) {
                printf("%g, %g,\n", waypoints[i][0], waypoints[i][1]);
            }
            printf("]\n");

            // Compute the two closest waypoints
            std::vector<std::vector<double> > dist_wps;
            std::vector<std::vector<double> > dist_wps_sort;
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

            dist_wps_sort = dist_wps;
            
            std::sort(dist_wps_sort.begin(), dist_wps_sort.end() - 1, sortcol);
            printf("dist_wps_sort = [\n");
            for (unsigned long i = 0; i < waypoints.size() - 1; i++) {
                printf("%g, %g\n", dist_wps_sort[i][0], dist_wps_sort[i][1]);
            }
            printf("]\n");

            double second_smallest[3] = {waypoints[dist_wps_sort[1][1]][0], waypoints[dist_wps_sort[1][1]][1], dist_wps_sort[1][1]};
            double smallest[3]        = {waypoints[dist_wps_sort[0][1]][0], waypoints[dist_wps_sort[0][1]][1], dist_wps_sort[0][1]};

            ROS_INFO("The two closest waypoints to the vehicle:");
            ROS_INFO("smallest = [%g, %g]", smallest[0], smallest[1]);
            ROS_INFO("second_smallest = [%g, %g]", second_smallest[0], second_smallest[1]);
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);

            // Compute the first waypoint according to the direction of travel.
            // It returns the waypoint index and start_heading
            bool out = true;
            double dist_target = 0.0;
            double *behavior = choose_first_waypoint(smallest, second_smallest, dist_target, out);
            int idx_first_wp = (int) behavior[0];
            int idx_last_wp  = (int) behavior[1];
            double start_heading = behavior[2];
            delete[] behavior;
            ROS_INFO("start_heading_before = %g", start_heading*180/M_PI);
            ROS_INFO("out = %d", out);

            int wp_jump_min = 1;
            int wp_jump_max = 2;
            if (num_wp > 5) {
                if (num_wp > 7) wp_jump_max = 2;

                if (out == true) {
                    if (clockwise) {
                        if (dist_target < radius + delta_radius && num_wp > 5) {
                            // idx_first_wp = (idx_first_wp - wp_jump_max) % (waypoints.size() - 1);
                            int n = idx_first_wp - wp_jump_max;
                            idx_first_wp = ((n % M) + M) % M;
                            ROS_INFO("Index decreased by %d (CW) due to the closeness of the vehicle to the target", wp_jump_max);
                        }
                        else {
                            if (start_heading < start_heading_th_out) {
                                // idx_first_wp = (idx_first_wp - wp_jump_min) % (waypoints.size() - 1);
                                int n = idx_first_wp - wp_jump_min;
                                idx_first_wp = ((n % M) + M) % M;
                                ROS_INFO("Index decreased by 1 (CW) due to small start_heading");
                            }
                        }
                    }
                    else {
                        if (dist_target < radius + delta_radius && num_wp > 5) {
                            // idx_first_wp = (idx_first_wp + wp_jump_max) % (waypoints.size() - 1);
                            int n = idx_first_wp + wp_jump_max;
                            idx_first_wp = ((n % M) + M) % M;
                            ROS_INFO("Index increased by %d (CCW) due to the closeness of the vehicle to the target", wp_jump_max);
                        }
                        else {
                            if (start_heading < start_heading_th_out) {
                                // idx_first_wp = (idx_first_wp + wp_jump_min) % (waypoints.size() - 1);
                                int n = idx_first_wp + wp_jump_min;
                                idx_first_wp = ((n % M) + M) % M;
                                ROS_INFO("Index increased by 1 (CCW) due to small start_heading");
                            }
                        }
                    }
                }
                else {
                    if (clockwise) {
                        if (dist_target < radius + delta_radius && num_wp > 5) {
                            // idx_first_wp = (idx_first_wp - wp_jump_max) % (waypoints.size() - 1);
                            int n = idx_first_wp - wp_jump_max;
                            idx_first_wp = ((n % M) + M) % M;
                            ROS_INFO("Index decreased by %d (CW) due to the closeness of the vehicle to the target", wp_jump_max);
                        }
                        else {
                            if (start_heading < start_heading_th_in) {
                                // idx_first_wp = (idx_first_wp - wp_jump_min) % (waypoints.size() - 1);
                                int n = idx_first_wp - wp_jump_min;
                                idx_first_wp = ((n % M) + M) % M;
                                ROS_INFO("Index decreased by 1 (CW) due to small start_heading");
                            }
                        }
                    }
                    else {
                        if (dist_target < radius + delta_radius && num_wp > 5) {
                            // idx_first_wp = (idx_first_wp + wp_jump_max) % (waypoints.size() - 1);
                            int n = idx_first_wp + wp_jump_max;
                            idx_first_wp = ((n % M) + M) % M;
                            ROS_INFO("Index increased by %d (CCW) due to the closeness of the vehicle to the target", wp_jump_max);
                        }
                        else {
                            if (start_heading < start_heading_th_in) {
                                // idx_first_wp = (idx_first_wp + wp_jump_min) % (waypoints.size() - 1);
                                int n = idx_first_wp + wp_jump_min;
                                idx_first_wp = ((n % M) + M) % M;
                                ROS_INFO("Index increased by 1 (CCW) due to small start_heading");
                            }
                        }
                    }
                }
            }

            if (num_wp > 7) {
                // Also, increase/decrease index if the vehicle is to close to the first waypoint in the circle
                double vect_goal[2] = {waypoints[idx_first_wp][0] - vehicle_pos[0], waypoints[idx_first_wp][1] - vehicle_pos[1]};
                double dist_first_wp = sqrt(pow(vect_goal[0], 2) + pow(vect_goal[1], 2));
                if (dist_first_wp < arc_len_th) {
                    if (clockwise) {
                        // Decrease index to the next waypoint in the correct direction
                        // idx_first_wp = (idx_first_wp - wp_jump_min) % (waypoints.size() - 1);
                        int n = idx_first_wp - wp_jump_min;
                        idx_first_wp = ((n % M) + M) % M;
                        ROS_INFO("idx_first_wp was decreased by %d (CW) due to the closeness of \
                                the vehicle to the first wp of the circle", wp_jump_min);
                    }
                    else {
                        // idx_first_wp = (idx_first_wp + wp_jump_min) % (waypoints.size() - 1);
                        int n = idx_first_wp + wp_jump_min;
                        idx_first_wp = ((n % M) + M) % M;
                        ROS_INFO("idx_first_wp was by %d (CCW) increased due to the closeness of \
                                the vehicle to the first wp of the circle", wp_jump_min);
                    }
                }
            }

            // Redefine the list of waypoints according to the new criteria based on
            // closest waypoint from vehicle and direction of circling (cloclwise or counter-cw)
            int j, k, n, m = 0;
            unsigned long last_wp = 0;
            geometry_msgs::Pose elem;
            geometry_msgs::PoseArray circular_path;
            if (clockwise) {
                for (unsigned long i = 0; i < waypoints.size(); i++) {
                    n = idx_first_wp - i;
                    j = ((n % M) + M) % M;
                    elem.position.x = waypoints[j][0];
                    elem.position.y = waypoints[j][1];
                    elem.orientation.z = 0.0;
                    elem.orientation.w = 1.0;
                    circular_path.poses.push_back(elem);
                    if (j == idx_last_wp && return_wp == -1) {
                        last_wp = i;
                        break;
                    }
                    else if (j == return_wp) {
                        last_wp = i;
                        break;
                    }
                }
            }
            else {
                for (unsigned long i = 0; i < waypoints.size(); i++) {
                    m = idx_first_wp + i;
                    k = ((m % M) + M) % M;
                    elem.position.x    = waypoints[k][0];
                    elem.position.y    = waypoints[k][1];
                    elem.orientation.z = 0.0;
                    elem.orientation.w = 1.0;
                    circular_path.poses.push_back(elem);
                    if (k == idx_last_wp && return_wp == -1) {
                        last_wp = i;
                        break;
                    }
                    else if (k == return_wp) {
                        last_wp = i;
                        break;
                    }
                }
            }
            printf("circular_path_enu = [\n");
            // for (unsigned long i = 0; i < waypoints.size(); i++) {
            for (unsigned long i = 0; i < last_wp + 1; i++) {
                printf("%g, %g\n", circular_path.poses[i].position.x, circular_path.poses[i].position.y);
            }
            printf("]\n");

            // Now let's convert the circular path to NED in order to properly send
            // it to the path-planner in path_to_goal.cpp
            geometry_msgs::Pose el;
            geometry_msgs::PoseArray circular_ned;
            // for ( unsigned long i = 0; i < waypoints.size(); i++) {
            for ( unsigned long i = 0; i < last_wp + 1; i++) {
                el.position.x = circular_path.poses[i].position.y;
                el.position.y = circular_path.poses[i].position.x;
                circular_ned.poses.push_back(el);
            }

            res.circle_path = circular_ned;
            return true;
        }

    public:
        double* choose_first_waypoint(double smallest[3], double sec_smallest[3], double &dist_target, bool &out)
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
            dist_target = mag_vect_target;

            if (mag_vect_target > radius) {
                out = true;
                ROS_INFO("The vehicle is out of the circle");
            }
            else {
                out = false;
                ROS_INFO("The vehicle is inside the circle");
            }
            ROS_INFO("Is the vehicle outside the circle? => %d", out);
            ROS_INFO("vehicle_pos = [%g, %g]", vehicle_pos[0], vehicle_pos[1]);
            ROS_INFO("mag_vehicle_target = %g", mag_vect_target);

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
                    ret[1] = (double) sec_smallest[2];
                    ret[2] = start_heading;
                    ROS_INFO("We've picked the smallest-clockwise = [%g, %g], index = %d", smallest[0], smallest[1], (int) smallest[2]);
                    return ret;
                }
                else if (reject_2smallest > 0) {
                    double num = vect_2smallest[0]*vect_target[0] + vect_2smallest[1]*vect_target[1];
                    double den = mag_vect_2smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) sec_smallest[2];
                    ret[1] = (double) smallest[2];
                    ret[2] = start_heading;
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
                    ret[1] = (double) sec_smallest[2];
                    ret[2] = start_heading;
                    ROS_INFO("We've picked the smallest-counter-cw = [%g, %g], index = %d", smallest[0], smallest[1], (int) smallest[2]);
                    ROS_INFO("out = %d", out);
                    return ret;
                }
                else if (reject_2smallest < 0) {
                    double num = vect_2smallest[0]*vect_target[0] + vect_2smallest[1]*vect_target[1];
                    double den = mag_vect_2smallest*mag_vect_target;
                    start_heading = acos(num/den);
                    ret[0] = (double) sec_smallest[2];
                    ret[1] = (double) smallest[2];
                    ret[2] = start_heading;
                    ROS_INFO("We've picked the second-smallest-counter-cw = [%g, %g], index = %d", sec_smallest[0], sec_smallest[1], (int) sec_smallest[2]);
                    return ret;
                }
                else {
                    ROS_ERROR("Something went wrong computing the rejection component");
                }
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circular_path");
    CircleMe circleMe;
    ros::spin();
    return 0;
}


                





