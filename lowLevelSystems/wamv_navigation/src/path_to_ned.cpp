#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Path.h>
#include "custom_messages_biggie/waypoint_array.h"
#include <vector>

/* This node listens for the global_planner message in the "/move_base/NavfnROS/plan" 
 * topic, as well as for the "ned_origin" transform.  Using these two, it transforms the
 * global trajectory from odom_enu to odom_ned reference frame. */

class PathToNED 
{
    private:
        message_filters::Subscriber<nav_msgs::Path> map_path_sub;
        tf::TransformListener listener;
        tf::MessageFilter<nav_msgs::Path> *tf_filter;
        ros::NodeHandle node;
        ros::Publisher path_ned_pub;
        int count;
        bool latch = true;
        
    public:
        PathToNED()
        {
            map_path_sub.subscribe(node, "/move_base/NavfnROS/plan", 100);
            // tf_filter = new tf::MessageFilter<nav_msgs::Path>(map_path_sub, listener, "odom_ned", 100);
            tf_filter = new tf::MessageFilter<nav_msgs::Path>(map_path_sub, listener, "ned_origin", 10);
            tf_filter->registerCallback(boost::bind(&PathToNED::msgCallback, this, _1));
            // path_ned_pub = node.advertise<custom_messages_biggie::waypoint_array>("/waypoint_array", 10);
            path_ned_pub = node.advertise<custom_messages_biggie::waypoint_array>("/waypoint_array", 10, latch);
            count = 0;
        }

    private:
        void msgCallback(const nav_msgs::Path::ConstPtr &path_ptr)
        {
            int size_ned = 0;
            count++;
            if (count == 1) {
                printf("*** got message ***\n");
                geometry_msgs::PointStamped point_out;
                geometry_msgs::PointStamped point_ptr;
                custom_messages_biggie::waypoint_array path_ned;
                int sz = path_ptr->poses.size();
                double segment = 5.0;
                double acumul = 0.0;
                double trav = 0.0;
                double prev_point[2] = {path_ptr->poses[0].pose.position.x,
                                        path_ptr->poses[0].pose.position.y};

                printf("path_size = %u\n", sz);

                for (int i = 1; i < sz; i++) {
                    try {
                        // Downsample the path according to the 'segment' parameter
                        if (trav < segment) {
                            // 'trav' is the traveled distance over current segment so far
                            trav = sqrt(pow(prev_point[0] - path_ptr->poses[i].pose.position.x, 2) +
                                        pow(prev_point[1] - path_ptr->poses[i].pose.position.y, 2));
                        }
                        if ((i == sz - 1) || (trav >= segment)) {
                            if (i == sz - 1) {
                                printf("got to last pose of the path, at i = %d\n", i);
                            }
                            // printf("trav = %g, i = %d\n", trav, i);
                            point_ptr.header.frame_id = "wamv/odom";
                            point_ptr.point.x = path_ptr->poses[i].pose.position.x;
                            point_ptr.point.y = path_ptr->poses[i].pose.position.y;
                            point_ptr.point.z = 0.0;

                            // listener.transformPoint("odom_ned", point_ptr, point_out);
                            listener.transformPoint("ned_origin", point_ptr, point_out);
                            
                            path_ned.waypoint_array.push_back(point_out.point.x);
                            path_ned.waypoint_array.push_back(point_out.point.y);
                            path_ned.waypoint_array.push_back(1.5);

                            prev_point[0] = path_ptr->poses[i].pose.position.x;
                            prev_point[1] = path_ptr->poses[i].pose.position.y;

                            size_ned++;
                            trav = 0.0;
                        }
                    }
                    catch (tf::TransformException &ex) {
                        printf("Failure %s\n", ex.what());  // print exception which was caught
                    }
                }
                // Debugging printouts
                printf("last original waypoint_enu = [%g, %g]\n", path_ptr->poses[sz-1].pose.position.x, path_ptr->poses[sz-1].pose.position.y); 
                printf("last waypoint_enu_ds  = [%g, %g]\n", point_ptr.point.x, point_ptr.point.y);
                printf("last waypoint_ned_ds = [%g, %g]\n", point_out.point.x, point_out.point.y);
                printf("path_ned size = %u\n", size_ned); 
                path_ned_pub.publish(path_ned);
                count = 0;
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_to_ned");
    PathToNED pathToned;
    ros::spin();
    return 0;
}


                




