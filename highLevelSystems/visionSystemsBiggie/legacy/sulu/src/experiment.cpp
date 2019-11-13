//Original Author:Travis Moscicki
//Date:4/16/2016
//Last Edit:5/11/2016 - Travis Moscicki

//The purpose of this script is to capture image data about different colored objects
//by selecting a region of interest in the frame, converting this to a number of
//different color spaces, and then logging said data.

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <math.h>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Point32.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "image_geometry/pinhole_camera_model.h"
#include "sulu/vision.h"

#include <tf/transform_listener.h>

using namespace std;
using namespace cv;
using namespace boost;

class coord
{
public:
	coord();
	void set(double x, double y);
	double xPos;
	double yPos;
};
coord::coord()
{
	xPos=0;
	yPos=0;
}
void coord::set(double x, double y)
{
	xPos=x;
	yPos=y;
}

int drag = 0;
bool boxCounter=false;
Point point1, point2;
vector<Rect> rect;
int numSamples=0;
bool theFlag=false;
int deltaY;
int deltaX;
vector<int> numPixels;

Mat distanceMat(480, 640, CV_32FC1);
Mat angleMat(480, 640, CV_32FC1);

int maxNumberObjects=10;
int minObjectArea=800;
int maxObjectArea=100000;
float maxDistance=7.0;
vector<coord> objVec;
bool objectFound;

Mat greyEdge(Mat &cameraFeed);
void mouseHandler(int event, int x, int y, int flags, void* param);
void trackObject(Mat &cameraFeed, Mat distanceMat);
void drawObject(Mat &cameraFeed);

typedef sulu::vision Vision;

namespace rotationTheta
{
	double theta;
}

class passCloud
{
	image_geometry::PinholeCameraModel cam_model_;

	public:
		vector<Point2d> imagePoints;
		passCloud (ros::NodeHandle& nh) : nh_(nh)
		{
			pointSub = nh_.subscribe("/laser_sweeps", 1, &passCloud::callback,this);
			infoSub = nh_.subscribe("/camera_info", 1, &passCloud::process,this);
		} // your constructor and store the passed node handle (nh) to your class attribute nh_

		void callback (const sensor_msgs::PointCloud::ConstPtr& msg)
		{
			pointsRos.clear();
			imagePoints.clear();
			pointsRos = msg->points; // copy the variable that the callback passes in to you class variable (attribute) input

			nh_.param<double>("rotationTheta/theta", rotationTheta::theta, 0);
			float p = rotationTheta::theta;

			points.resize(pointsRos.size());
			for(int i = 0; i<pointsRos.size(); i++)
			{
				//cout << "the passed point is: " << pointsRos[i] << endl;
				float tempX = pointsRos[i].x;
				float tempY = pointsRos[i].y;
				float tempZ = pointsRos[i].z;
				points[i].x = tempY;
				points[i].y = tempZ*cos(p)-tempX*sin(p);
				points[i].z = tempZ*sin(p)+tempX*cos(p);
			}

			Mat newMatrix(480, 640, CV_32FC3);
			Mat tempMatrix[3];
			split(newMatrix, tempMatrix);

			for (int i = 0; i<points.size(); i++)
			{
				imagePoints.push_back(cam_model_.project3dToPixel(points[i]));
				//cout<<imagePoints[i].y<<endl;
				imagePoints[i].y = 240-imagePoints[i].y;
				if(imagePoints[i].y>-200&&imagePoints[i].y<279&&imagePoints[i].x>2&&imagePoints[i].x<658)
				{
					if(sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x)<maxDistance)
					{
						//tempMatrix[0].at<float>(imagePoints[i].y-2, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x-2) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//tempMatrix[0].at<float>(imagePoints[i].y-1, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x-1) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						tempMatrix[0].at<float>(imagePoints[i].y+290, imagePoints[i].x-25) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//cout<<"The atan function is " << ((atan2(points[i].z, points[i].x)*180.0/M_PI))<<endl;
						tempMatrix[1].at<float>(imagePoints[i].y+290, imagePoints[i].x-25) = ((atan2(points[i].x, points[i].z)* 180.0 /M_PI)+90.0);
						//tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x+1) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//tempMatrix[0].at<float>(imagePoints[i].y+1, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x+2) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
						//tempMatrix[0].at<float>(imagePoints[i].y+2, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					}
					//tempMatrix[1].at<float>(imagePoints[i].y-2, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y-1, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y+290, imagePoints[i].x-20) = atan2(points[i].z, points[i].x)* 180 /M_PI;
					//tempMatrix[1].at<float>(imagePoints[i].y+1, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y+2, imagePoints[i].x) = atan2(points[i].x, points[i].z);
				}
			}
			cv::flip(tempMatrix[0], tempMatrix[0], 1);
			cv::flip(tempMatrix[1], tempMatrix[1], 1);

			Mat dilateElement = getStructuringElement(MORPH_RECT, Size(25, 25));
			Mat erodeElement = getStructuringElement(MORPH_RECT, Size(15, 15));

			dilate(tempMatrix[0], tempMatrix[0], dilateElement);
			erode(tempMatrix[0], tempMatrix[0], erodeElement);

			dilate(tempMatrix[1], tempMatrix[1], dilateElement);
			erode(tempMatrix[1], tempMatrix[1], erodeElement);

			distanceMat = tempMatrix[0];
			angleMat = tempMatrix[1];
		}

		void process(const sensor_msgs::CameraInfoConstPtr& msg)
	  	{
	 		tf::StampedTransform transform;
			cam_model_.fromCameraInfo(msg);
		}
	protected:
	ros::NodeHandle nh_;
	vector<geometry_msgs::Point32> pointsRos;
	ros::Subscriber pointSub;
	ros::Subscriber infoSub;
	int i;
	vector<Point3f> points;
};

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	public:
	Mat cameraFeedRos;
	ImageConverter(ros::NodeHandle& nh) : nh_(nh), it_(nh_)
	{
		image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
	}
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cameraFeedRos = cv_ptr->image;

	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "experiment");
	ros::NodeHandle nh;

	ImageConverter* ic = 0;
	ic = new ImageConverter(nh);

	passCloud* node = 0;
	node = new passCloud(nh);

	sulu::vision distanceAndAngle;
	ros::Publisher visionPub=nh.advertise<sulu::vision>("visionMSG", 10);
	ros::Publisher visionPub2=nh.advertise<sulu::vision>("visionMSGSecond", 10);

	ros::Rate loop_rate(4);

	Mat cameraFeed;
	VideoWriter camWriter("cameraFeed.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(640, 480), true);
	//VideoWriter lidarWriter("lidar.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(640, 480), true);
	while(ros::ok())
	{
		objVec.clear();
		while(!cameraFeed.rows)
		{
			cameraFeed = ic->cameraFeedRos;
			ros::spinOnce();
		}
		//blur(cameraFeed, cameraFeed, Size(12,12));
       		//namedWindow("cameraFeed", CV_WINDOW_AUTOSIZE);
       		//namedWindow("lidar", CV_WINDOW_AUTOSIZE);
        	//namedWindow("angle", CV_WINDOW_AUTOSIZE);

		trackObject(cameraFeed, distanceMat);

		Mat greyEdgeMatrix;

		greyEdgeMatrix=greyEdge(cameraFeed);
		//cvtColor(cameraFeed, greyEdgeMatrix, COLOR_BGR2YCrCb);
		Mat greyChannels[3];
		split(greyEdgeMatrix, greyChannels);
		float theAverageChan0;
		float theAverageChan1;
		float theAverageChan2;
		float theAverageChan3;
		float theAverageChan4;
		float theSumChan0;
		float theSumChan1;
		float theSumChan2;
		float theSumChan3;
		float theSumChan4;
		//vector<sulu::vision> smallestDisVals;
		//smallestDisVals.clear();
		//now that we found an object, output the values to a script
			Vision smallest, secondSmallest;
			smallest.distance=100;
			secondSmallest.distance=100;

		if(objectFound)
		{
			for (int i=0; i<objVec.size()-1; i++)
			{
				float count = 0.0;
				theSumChan0=0.0;
				for(int j=objVec[i].yPos-5; j<objVec[i].yPos+5; j++)
				{
					for(int k=objVec[i].xPos-5; k<objVec[i].xPos+5; k++)
					{
						theSumChan0+=distanceMat.at<float>(j, k);
					}
				}
				theSumChan1=0.0;
				for(int j=objVec[i].yPos-5; j<objVec[i].yPos+5; j++)
				{
					for(int k=objVec[i].xPos-5; k<objVec[i].xPos+5; k++)
					{
						//cout << angleMat.at<float>(j,k)<<endl;
						theSumChan1+=angleMat.at<float>(j, k);
						count++;
					}
				}
				theSumChan2=0.0;
				for(int j=objVec[i].yPos-5; j<objVec[i].yPos+5; j++)
				{
					for(int k=objVec[i].xPos-5; k<objVec[i].xPos+5; k++)
					{
						int var=(int)greyChannels[0].at<unsigned char>(j, k);
						theSumChan2+=var;
					}
				}
				theSumChan3=0.0;
				for(int j=objVec[i].yPos-5; j<objVec[i].yPos+5; j++)
				{
					for(int k=objVec[i].xPos-5; k<objVec[i].xPos+5; k++)
					{
						int var=(int)greyChannels[1].at<unsigned char>(j, k);
						theSumChan3+=var;
					}
				}
				theSumChan4=0.0;
				for(int j=objVec[i].yPos-5; j<objVec[i].yPos+5; j++)
				{
					for(int k=objVec[i].xPos-5; k<objVec[i].xPos+5; k++)
					{
						int var=(int)greyChannels[2].at<unsigned char>(j, k);
						theSumChan4+=var;
					}
				}
				//cout << "The count is: " << count << endl;
				theAverageChan0=theSumChan0/count;
				//cout << "theSumChan1:" << theSumChan1 << endl;
				theAverageChan1=theSumChan1/count;
				theAverageChan2=theSumChan2/count;
				theAverageChan3=theSumChan3/count;
				theAverageChan4=theSumChan4/count;
				//cout << "The average distance is:" << theAverageChan0 << " and the object # is:" << i << endl;
				distanceAndAngle.distance=theAverageChan0;
				//cout << "The average angle is:" << theAverageChan1 << " and the object # is:" << i << endl;
				distanceAndAngle.angle=theAverageChan1;
				//cout << "the angle is "<< theAverageChan1 << endl;
				//cout << "The average value is:" << theAverageChan2 << " and the object # is:" << i << endl;
				//cout << "The average value is:" << theAverageChan3 << " and the object # is:" << i << endl;
				//cout << "The average value is:" << theAverageChan4 << " and the object # is:" << i << endl;
				//visionPub.publish(distanceAndAngle);
			if(smallest.distance > distanceAndAngle.distance){
				secondSmallest=smallest;
				smallest = distanceAndAngle;
			}
			//cout<<"Smallest is " << smallest.distance << " and " << smallest.angle << endl;
			//cout<<"Second Smallest is " << secondSmallest.distance << "and " <<secondSmallest.angle << endl;
			visionPub.publish(smallest);
			visionPub2.publish(secondSmallest);
			}
		}
		camWriter.write(cameraFeed);
		//lidarWriter.write(distanceMat);
		//imshow("cameraFeed", cameraFeed);
		//imshow("lidar", distanceMat);
		//imshow("angle", angleMat);
		cameraFeed.rows=0;
		int esc = cvWaitKey(20);
		if( esc == 1048603)
		{
			break;
		}
	}
	return 0;
}

Mat greyEdge(Mat &cameraFeed)
{
	float divideBy;
	Scalar theSum;
	double illuminationGreen;
	double illuminationBlue;
	double illuminationRed;
	Mat camChannels[3];
	Mat camChannels2[3];
	double blueSum;
	double greenSum;
	double redSum;
	Mat sumChannels;
	blueSum=greenSum=redSum=0;
	const int numberPixels = 640*480;
	Mat normalized;

	split(cameraFeed, camChannels);
	split(cameraFeed, camChannels2);

	for (int y = 1; y < cameraFeed.rows; y++)
	{
		for (int x = 1; x < cameraFeed.cols; x++)

		{
			Point2f location(x, y);
			double dy, dx;
			dy=(camChannels[0].at<uchar>(location.y+1, location.x)-camChannels[0].at<uchar>(location.y-1, location.x))/2;
			dx=(camChannels[0].at<uchar>(location.y, location.x+1)-camChannels[0].at<uchar>(location.y, location.x-1))/2;
			blueSum+=pow(sqrt(dy*dy+dx*dx),16);
			//blueSum+=sqrt(dy*dy+dx*dx);
			//blueGrad.at<uchar>(location.y, location.x) = sqrt(dy*dy+dx*dx);
			//cout << num<< endl;

		}
	}
	for (int y = 1; y < cameraFeed.rows; y++)
	{
		for (int x = 1; x < cameraFeed.cols; x++)

		{
			Point2f location(x, y);
			double dy, dx;
			dy=(camChannels[1].at<uchar>(location.y+1, location.x)-camChannels[1].at<uchar>(location.y-1, location.x))/2;
			dx=(camChannels[1].at<uchar>(location.y, location.x+1)-camChannels[1].at<uchar>(location.y, location.x-1))/2;
			greenSum+=pow(sqrt(dy*dy+dx*dx),16);
			//greenSum+=sqrt(dy*dy+dx*dx);
			//cout << num<< endl;

		}
	}

	for (int y = 1; y < cameraFeed.rows; y++)
	{
		for (int x = 1; x < cameraFeed.cols; x++)
		{
			Point2f location(x, y);
			double dy, dx;
			dy=(camChannels[2].at<uchar>(location.y+1, location.x)-camChannels[2].at<uchar>(location.y-1, location.x))/2;
			dx=(camChannels[2].at<uchar>(location.y, location.x+1)-camChannels[2].at<uchar>(location.y, location.x-1))/2;
			redSum+=pow(sqrt(dy*dy+dx*dx),16);
			//redSum+=sqrt(dy*dy+dx*dx);
			//cout << num<< endl;

		}
	}

	illuminationBlue = pow(blueSum/numberPixels, 1/16.0);
	illuminationGreen = pow(greenSum/numberPixels, 1/16.0);
	illuminationRed = pow(redSum/numberPixels, 1/16.0);

	Scalar blue = (sum(camChannels2[0]));
	double blueVal = (blue[0]/numberPixels);

	Scalar greenChan = (sum(camChannels2[1]));
	double greenVal = (greenChan[0]/numberPixels);

	Scalar redChan = (sum(camChannels2[2]));
	double redVal = (redChan[0]/numberPixels);

	for (int y = 1; y < cameraFeed.rows; y++)
	{
		for (int x = 1; x < cameraFeed.cols; x++)
		{
		Point2f location(x, y);
		camChannels2[0].at<uchar>(location.y, location.x)=camChannels2[0].at<uchar>(location.y, location.x)*(illuminationBlue/blueVal);
		//cout << num<< endl;
		}
	}
	for (int y = 1; y < cameraFeed.rows; y++)
	{
		for (int x = 1; x < cameraFeed.cols; x++)

		{
		Point2f location(x, y);
		camChannels2[1].at<uchar>(location.y, location.x)=camChannels2[1].at<uchar>(location.y, location.x)*(illuminationGreen/greenVal);
		//cout << num<< endl;
		}
	}
	for (int y = 1; y < cameraFeed.rows; y++)
	{
		for (int x = 1; x < cameraFeed.cols; x++)
		{
		Point2f location(x, y);
		camChannels2[2].at<uchar>(location.y, location.x)=camChannels2[2].at<uchar>(location.y, location.x)*(illuminationRed/redVal);
		//cout << num<< endl;
		}
	}
	merge(camChannels2, 3, normalized);
	return normalized;
}

void trackObject(Mat &cameraFeed, Mat distanceMat)
{
	//sets up temporary location for thresholded image
	coord loc;
	int count=0;
	//these two vectors are needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	distanceMat.convertTo(distanceMat, CV_32SC1);
	findContours(distanceMat, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//use moments method to find our filtered object
	double refArea = 0;
	objectFound = false;
	if (hierarchy.size() > 0)
	{
		//ensures we are not tracking too many objects due to a noisy filter
		int numObjects = hierarchy.size();
		//cout << numObjects << ", " << maxNumberObjects << endl;
		if (numObjects<maxNumberObjects)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				//openCV class "Moments" for information about location and size
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//thresholds out small objects
				if (area>minObjectArea && area<maxObjectArea)
				{
					double xPos = moment.m10 / area;
					double yPos = moment.m01 / area;
					loc.set(xPos, yPos);

					objVec.push_back(loc);
					drawObject(cameraFeed);
					objectFound = true;
				}
				else objectFound = false;
			}

		}
		else putText(cameraFeed, "there are just toooo many objects!", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}

void drawObject(Mat &cameraFeed)
{
	for (unsigned int i = 0; i<objVec.size()-1; i++)
	{
		cv::circle(cameraFeed, cv::Point(objVec[i].xPos, objVec[i].yPos), 10, cv::Scalar(0, 0, 0));
		cv::putText(cameraFeed, "object found!", cv::Point(objVec[i].xPos, objVec[i].yPos + 20), 1, 1, Scalar(0, 255, 0));
	}
}
