/*
Uses process from 2016 VBAS sulu package
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point32.h"
#include "usv_ahc_py/depth_points.h"
#include "usv_ahc_py/cluster.h"
#include "usv_ahc_py/cluster_list.h"
#include <vector> 
#include <cv_bridge/cv_bridge.h>
#include "image_geometry/pinhole_camera_model.h"
#include "rgbd_fusion/segment.h"


#define PI 3.14159265359

using namespace std;

//useful in many functions
vector<float> camera_transform;
vector<float> camera_rotation;
int res_x;
int res_y;
float theta;

bool camSettings = false;
bool isCloud = false;

usv_ahc_py::cluster clust;

image_geometry::PinholeCameraModel cam_model;

bool inFrame(geometry_msgs::Point32 point);
bool inFrame(float x, float y, float z);
cv::Point toCameraCoords(float x, float y, float z);
cv::Rect formatMask(vector<cv::Point> points);
cv::Rect generateMask(usv_ahc_py::cluster cluster);


float degToRad(float deg){return deg*(PI/180);}
float radToDeg(float rad){return rad*(180/PI);}

bool segmentCallback(rgbd_fusion::segment::Request  &req,
	 rgbd_fusion::segment::Response &res)
{
  //loads cluster in
  clust = req.clust;

  //loads camera settings in
  cam_model.fromCameraInfo(req.cam_info);
  res_x = req.cam_info.width;
  res_y = req.cam_info.height;

  cv::Rect rect = generateMask(clust);     

  //sets message data
  geometry_msgs::Point32 rootPoint;
  rootPoint.x = rect.x;
  rootPoint.y = rect.y;
  geometry_msgs::Point32 widthHeight;
  widthHeight.x = rect.width;
  widthHeight.y = rect.height;

  res.root_point = rootPoint;
  res.width_height = widthHeight;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segment");

  ros::NodeHandle nh; 
  
  nh.getParam("/camera_transform", camera_transform);
  nh.getParam("/camera_rotation", camera_rotation);
 
  ros::ServiceServer service = nh.advertiseService("RGBD_segmentation", segmentCallback);
  ROS_INFO("Segmentation Service Running");
  ros::spin();
     	 
  return 0;
}


cv::Point toCameraCoords(float x, float y, float z)
{
  cv::Point temp;

  cv::Point3d point; 

  //rotates points  //not sure if rotation value is correct
  point.x = y+camera_transform[0];
  point.y = z*cos(camera_rotation[0])-x*sin(camera_rotation[0])+camera_transform[1];
  point.z = -z*sin(camera_rotation[0])-x*cos(camera_rotation[0])+camera_transform[2];

  //gets camera points
  cv::Point2d imagePoint = cam_model.project3dToPixel(point);

  return imagePoint;
}


//use vector to centriod to make sure its within fov; atan2
bool inFrame(float x, float y, float z)
{
  //filter out any value that is not in front of the vehicle
  if(x <=0)
    return false;
  return true;
}

cv::Rect generateMask(usv_ahc_py::cluster cluster)
{
  vector<cv::Point> points; 
  for(int i = 0; i < cluster.raw_cluster.size(); i++)
  { 
    //need to plot point on image
    float x, y, z;
    x = cluster.raw_cluster[i].labeled_point[0];
    y = cluster.raw_cluster[i].labeled_point[1];
    z = cluster.raw_cluster[i].labeled_point[2];
    if(inFrame(x,y,z))
    {
      points.push_back(toCameraCoords(x,y,z));
    }
  }
  if(points.size() > 3)
  {
    cv::Rect rect = formatMask(points);
    return rect;
  }
  else 
  {
    return cv::Rect(-999,-999,-999,-999);
  }
}

cv::Rect formatMask(vector<cv::Point> points)
{
  int top, left, right, bottom;
  top = points[0].y;
  left = points[0].x;
  bottom = points[0].y;
  right = points[0].x;

  //gets bounding box of points
  for(int i = 0; i < points.size(); i ++)
  {
    if(points[i].y < top)
    {
      top = points[i].y;
    }
    else if(points[i].y > bottom)
    {
      bottom = points[i].y;
    }
    else if(points[i].x < left)
    {
      left = points[i].x;
    }
    else if(points[i].x > right)
    {
      right = points[i].x;
    }
  }

  return cv::Rect(left,top,right-left,bottom-top);
}
