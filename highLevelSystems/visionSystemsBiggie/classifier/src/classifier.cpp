#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point32.h"
#include "usv_ahc_py/depth_points.h"
#include "usv_ahc_py/cluster.h"
#include "usv_ahc_py/cluster_list.h"
#include <vector> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_geometry/pinhole_camera_model.h"
#include "rgbd_fusion/segment.h"
#include "color_classification/color_classification.h"

using namespace std;

vector<float> camera_transform;
vector<float> camera_rotation;

bool isCloud = false;
bool isImage = false;
bool camSettings = false;

cv::Mat img;
sensor_msgs::CameraInfo cam_info;
vector<usv_ahc_py::cluster> clusters;


void clusterCallback(const usv_ahc_py::cluster_list msg)
{  
  clusters.clear();
  for(int i = 0; i < msg.cluster_list.size(); i++)
  {
    clusters.push_back(msg.cluster_list[i]);
  }
  isCloud = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr img_ptr;
  img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  img = img_ptr -> image;
  isImage = true;
}

void camCallback(const sensor_msgs::CameraInfo msg)
{
  cam_info = msg;
  camSettings = true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "classifier");

	ros::NodeHandle nh;

  nh.getParam("/camera_transform", camera_transform);
  nh.getParam("/camera_rotation", camera_rotation);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imageSub = it.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 5, imageCallback);
  ros::Subscriber camInfoSub = nh.subscribe("/wamv/sensors/cameras/front_left_camera/camera_info",10,camCallback);
  ros::Subscriber clusterSub = nh.subscribe("/persistanceClusterList", 5, clusterCallback);

  ros::ServiceClient client = nh.serviceClient<rgbd_fusion::segment>("RGBD_segmentation");

  ROS_INFO("Waiting for segmentation service");
  ROS_INFO("Waiting for color classification service");

  while(ros::ok())
  {
    ros::spinOnce();

    rgbd_fusion::segment getMask;
    color_classification::color_classification getColor;

    for(int i = 0; i < clusters.size(); i++)
    {
      ROS_INFO_STREAM("NumPoints: " << clusters[i].raw_cluster.size());
      getMask.request.clust = clusters[i];
      getMask.request.cam_info = cam_info;

      //vector<rgbd_fusion::object_image> masks;

      if(client.call(getMask))
      {
        geometry_msgs::Point32 rootPoint = getMask.response.root_point;
        geometry_msgs::Point32 widthHeight = getMask.response.width_height;
        
        //escape system for non real values
        if(rootPoint.x = -999)
        {
          break; 
        }

        ROS_INFO_STREAM("X: "<< rootPoint.x << "\tY: " << rootPoint.y << "\tWidth: " << widthHeight.x << "\tHeight: " << widthHeight.y);

        cv::Rect temp(rootPoint.x, rootPoint.y, widthHeight.x, widthHeight.y);

        //crops image
        cv::Mat cropped = img(temp);
        
        //allows for transport
        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg; 
        std_msgs::Header header; // empty header
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cropped);
        img_bridge.toImageMsg(img_msg);

        getColor.request.image = img_msg;
        
        if(client.call(getColor))
        {
          vector<float> conf = getColor.response.confidence;
          vector<string> colors = getColor.response.colors;
          for(int i = 0; i < conf.size(); i++)
          {
            ROS_INFO_STREAM(colors[i] << ": " << conf[i]);
          }
        }

      }
    }

  }
 

  return 0;

}
