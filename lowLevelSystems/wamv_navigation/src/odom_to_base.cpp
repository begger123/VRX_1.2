#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/* This node gets data needed from the "/ship/state" topic in order to publish
 * the odometry-related topic as well as the associated transform, both with the name "odom".
 * The odom topic and transform published are defined according to the ENU convention. */

class OdomToBase
{
    private:
        ros::NodeHandle node;
        ros::Subscriber state_sub;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_br;
        double heading_enu;

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            ros::Time current_time;
            current_time = ros::Time::now();
        
            tf::Transform transform;
            geometry_msgs::TransformStamped odom_trans;
            heading_enu = convert_heading(tf::getYaw(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                                                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)));
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_enu);

            /* populate the transform */
            // for quaternion-related data we'll only consider yaw motion only
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "wamv/odom";
            odom_trans.child_frame_id = "wamv/base_link";

            odom_trans.transform.translation.x = msg->pose.pose.position.y;
            odom_trans.transform.translation.y = msg->pose.pose.position.x;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            // send the transform
            odom_br.sendTransform(odom_trans);

            /* populate the odom message */
            // for quaternion-related data we'll only consider yaw motion only
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "wamv/odom";
        
            odom.pose.pose.position.x = msg->pose.pose.position.y;
            odom.pose.pose.position.y = msg->pose.pose.position.x;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
        
            // set velocity
            // linear velocites are given in BFF
            odom.child_frame_id = "wamv/base_link";
            odom.twist.twist.linear.x = msg->twist.twist.linear.x;
            odom.twist.twist.linear.y = -msg->twist.twist.linear.y;
            odom.twist.twist.angular.z = -msg->twist.twist.angular.z;
            
            // publish the odom message
            odom_pub.publish(odom);
        }

    public:
        OdomToBase()
        {
            state_sub = node.subscribe("/p3d_wamv_ned", 10, &OdomToBase::odomCallback, this);
            odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
        }

        double unwrap_angle(double x)
        {
            // This function recomputes the clockwise north-zero-heading to
            // be in the [0, 2*pi] range as opposed to [-pi, pi]
            x = fmod(x, 2*M_PI);
            if (x < 0)
                x += 2*M_PI;

            return x;
        }

        double convert_heading(double head)
        {
            // This function transforms from clockwise north-zero-heading
            // to counter-clockwise east-zero-heading
            head = unwrap_angle(head);
            head = 2*M_PI + M_PI/2 - head;
            head = fmod(head, 2*M_PI);
            return head;
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom2base");
    OdomToBase odom2Base;
    ros::spin();

    return 0;
}

        

