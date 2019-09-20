#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Path.h>
#include <usv16_msgs/Usv16Mission.h>
#include <vector>

/* This node listens for the global_planner message in the "/move_base/NavfnROS/plan" 
 * topic, as well as for the "odom_ned" transform.  Using these two, it transforms the
 * global trajectory from odom_enu to odom_ned reference frame. */

class PathToNED 
{
    private:
        message_filters::Subscriber<nav_msgs::Path> map_path_sub;
        tf::TransformListener listener;
        tf::MessageFilter<nav_msgs::Path> *tf_filter;
        ros::NodeHandle node;
        ros::Publisher world_path_pub;
        int count;
        
    public:
        PathToNED()
        {
            map_path_sub.subscribe(node, "/move_base/NavfnROS/plan", 10);
            tf_filter = new tf::MessageFilter<nav_msgs::Path>(map_path_sub, listener, "odom_ned", 10);
            tf_filter->registerCallback(boost::bind(&PathToNED::msgCallback, this, _1));
            world_path_pub = node.advertise<usv16_msgs::Usv16Mission>("path_ned", 50);
            count = 0;
        }

    private:
        void msgCallback(const nav_msgs::Path::ConstPtr &path_ptr)
        {
            count++;
            if(count == 2) {
                printf("*** got message ***\n");
                geometry_msgs::PointStamped point_out;
                usv16_msgs::Usv16Mission path_world;
                path_world.timeNow = ros::Time::now();
                int sz = path_ptr->poses.size();
                printf("path_size = %u\n", sz);

                for(int i = 0; i < sz; i++) {
                    try {
                        geometry_msgs::PointStamped point_ptr;
                        point_ptr.header.frame_id = "odom";
                        point_ptr.point.x = path_ptr->poses[i].pose.position.x;
                        point_ptr.point.y = path_ptr->poses[i].pose.position.y;
                        point_ptr.point.z = 0.0;

                        listener.transformPoint("odom_ned", point_ptr, point_out);

                        // printf("point_out.x = %f\n", point_out.point.x);
                        // printf("point_out.y = %f\n", point_out.point.y);
                        
                        path_world.data.push_back(point_out.point.x);
                        path_world.data.push_back(point_out.point.y);
                        path_world.data.push_back(1.50);
                    }
                    catch(tf::TransformException &ex) {
                        printf("Failure %s\n", ex.what());  // print exception which was caught
                    }
                }
                world_path_pub.publish(path_world);
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


                




