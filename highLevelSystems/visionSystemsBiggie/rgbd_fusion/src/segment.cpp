/*
Uses lots of code from 2016 VBAS sulu package
TODO: Add 3d point of centroid to message
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
#include "rgbd_fusion/object_image.h"
#include "rgbd_fusion/object_image_list.h"
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

vector<usv_ahc_py::cluster> clusters;

image_geometry::PinholeCameraModel cam_model;

bool inFrame(geometry_msgs::Point32 point);
bool inFrame(float x, float y, float z);
cv::Point toCameraCoords(float x, float y, float z);
vector<cv::Rect> generateMasks();
cv::Rect generateMask(vector<cv::Point> points);

float degToRad(float deg){return deg*(PI/180);}
float radToDeg(float rad){return rad*(180/PI);}

bool segmentCallback(rgbd_fusion::segment::Request  &req,
	 rgbd_fusion::segment::Response &res)
{
  //loads clusters in
  clusters.clear();
  clusters = req.list;

  //loads camera settings in
  cam_model.fromCameraInfo(req.cam_info);
  res_x = req.cam_info.width;
  res_y = req.cam_info.height;

  vector<cv::Rect> rects = generateMasks(); 
  vector<rgbd_fusion::object_image> objects;
      
	for(int i = 0; i < rects.size(); i++)
  {
	  rgbd_fusion::object_image obj;
	  obj.object_id = i;
	  geometry_msgs::Point32 temp;
	  temp.x = rects[i].x;
	  temp.y = rects[i].y;
	  obj.root_point = temp;
	  temp.x = rects[i].width;
	  temp.y = rects[i].height;
	  obj.width_height = temp;
	  objects.push_back(obj);
	
	}

  res.masks = objects;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segment");

  ros::NodeHandle nh; 
 
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


vector<cv::Rect> generateMasks()
{
  vector<cv::Rect> rects;
  for(int i = 0; i < clusters.size(); i++)
  {
    vector<cv::Point> points;  
    for(int j = 0; j < clusters[i].raw_cluster.size(); j++)
    { 
      //need to plot point on image
      float x, y, z, id;
      x = clusters[i].raw_cluster[j].labeled_point[0];
      y = clusters[i].raw_cluster[j].labeled_point[1];
      z = clusters[i].raw_cluster[j].labeled_point[2];
      id = clusters[i].raw_cluster[j].labeled_point[3];
      if(inFrame(x,y,z))
      {
        points.push_back(toCameraCoords(x,y,z));
      }
    }
    if(points.size() > 3)
    {
      cv::Rect rect = generateMask(points);
      rects.push_back(rect);
    }

  }
  return rects;
}

bool inFrame(float x, float y, float z)
{
  //filter out any value that is not in front of the vehicle
  if(x <=0)
    return false;
  return true;
}

cv::Rect generateMask(vector<cv::Point> points)
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
