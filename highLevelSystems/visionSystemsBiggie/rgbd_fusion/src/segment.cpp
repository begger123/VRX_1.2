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
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"

#define PI 3.14159265359

using namespace std;

//useful in many functions
vector<float> camera_transform;
vector<float> camera_rotation;
int res_x;
int res_y;
int fov;

bool camSettings = false;
bool isCloud = false;

usv_ahc_py::cluster clust;

image_geometry::PinholeCameraModel cam_model;

bool inFrame(float x, float y, float z);
cv::Point toCameraCoords(float x, float y, float z);
cv::Rect formatMask(vector<cv::Point> points);
cv::Rect generateMask(usv_ahc_py::cluster cluster);
bool validPoint(cv::Point temp);

//position of the wamv so that it only checks objects in camera frame
float wamvX;
float wamvY;
float wamvTheta;

bool segmentCallback(rgbd_fusion::segment::Request  &req,
	 rgbd_fusion::segment::Response &res)
{

  ROS_INFO("Attempting Segmentation");
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

  //ROS_INFO_STREAM("Segmented X: "<< rootPoint.x << "\tY: " << rootPoint.y << "\tWidth: " << widthHeight.x << "\tHeight: " << widthHeight.y);

  return true;
}

void odomCallback(const geometry_msgs::Pose2D msg)
{
  wamvTheta = msg.theta;
  wamvX = msg.x;
  wamvY = msg.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segment");

  ros::NodeHandle nh; 
  
  nh.getParam("/camera_transform", camera_transform);
  nh.getParam("/camera_rotation", camera_rotation);
  nh.getParam("/camera_fov", fov);
 
  ros::ServiceServer service = nh.advertiseService("RGBD_segmentation", segmentCallback);
  ros::Subscriber camInfoSub = nh.subscribe("/vehicle_pose",10,odomCallback);  //odometry message
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
//wamv and centroids are in ned
bool inFrame(float x, float y, float z)
{
  //only looking at x and y values
  //ROS_INFO_STREAM("WAMV-X: " << wamvLoc.x << "\tWAMV-Y: " << wamvLoc.y);
  //ROS_INFO_STREAM("Centroid-X: " <<  x << "\tCentroid-Y: " << y);
  float angle = atan2(x-wamvX, y-wamvY);
  float theta = fov/2 * (PI/180); //gets theta in radians
  
  if(angle < 0)
    angle += 2*PI;

  //generate boundry conditions
  float min, max;
  min = wamvTheta-theta;
  max = wamvTheta+theta;

  ROS_INFO_STREAM("Centroid Angle: "<< angle);
  ROS_INFO_STREAM("WAMV Angle: "<< wamvTheta);

  if(min >= 0 && max <=2*PI) //no boundry conditions
  {
    if(angle >= min && angle <= max)
    {
      return true;
    }
    else 
    {
      return false;
    }
  }
  else if(min < 0)
  {
    min += abs(min);
    max += abs(min);
    angle += abs(min);
    if(angle >2*PI)
      angle-=2*PI;

    if(angle >= min && angle <= max)
    {
      return true;
    }
    else 
    {
      return false;
    }
  }
  else if(max > 2*PI)
  {
    min -= max-(2*PI);
    max -= max-(2*PI);
    angle -= max-(2*PI);
    if(angle < 0)
      angle+=(2*PI);

    if(angle >= min && angle <= max)
    {
      return true;
    }
    else 
    {
      return false;
    }
  }
  
  //just in case somehow isn't caught
  return false;
}

cv::Rect generateMask(usv_ahc_py::cluster cluster)
{
  vector<cv::Point> points; 
  geometry_msgs::Point32 cent = cluster.centroid;

  if(inFrame(cent.x,cent.y,cent.z))
  {
    for(int i = 0; i < cluster.raw_cluster.size(); i++)
    { 
      //need to plot point on image
      float x, y, z;
      x = cluster.raw_cluster[i].labeled_point[0];
      y = cluster.raw_cluster[i].labeled_point[1];
      z = cluster.raw_cluster[i].labeled_point[2];

      cv::Point temp = toCameraCoords(x,y,z);
      if(validPoint(temp))
      {
        points.push_back(temp);
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

bool validPoint(cv::Point temp)
{
  if(temp.x > res_x || temp.x < 0)
    return false;
  if(temp.y > res_y || temp.y < 0)
    return false;
  return true;
}