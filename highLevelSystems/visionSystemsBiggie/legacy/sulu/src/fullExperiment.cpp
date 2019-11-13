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

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/Point32.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "image_geometry/pinhole_camera_model.h"

#include <tf/transform_listener.h>
#include "../includes/spline.h"

using namespace std;
using namespace cv;imagePoints
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

int maxNumberObjects=30;
int minObjectArea=100;
int maxObjectArea=10000;
vector<coord> objVec;
bool objectFound;

Mat greyEdge(Mat &cameraFeed);
void mouseHandler(int event, int x, int y, int flags, void* param);
void trackObject(Mat &cameraFeed, Mat distanceMat);
void drawObject(Mat &cameraFeed);

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

			nh_.param<double>("rotationTheta/theta", rotationTheta::theta, .15);
			float p = rotationTheta::theta;

			points.resize(pointsRos.size());
			for(int i = 0; i<pointsRos.size(); i++)
			{
				//cout << "the passed point is: " << pointsRos[i] << endl;
				float tempX = pointsRos[i].x;
				float tempY = pointsRos[i].y;
				float tempZ = pointsRos[i].z;

				cv::Point3d
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
				imagePoints[i].y = 240-imagePoints[i].y;
				if(imagePoints[i].y>2&&imagePoints[i].y<478&&imagePoints[i].x>2&&imagePoints[i].x<638)
				{
					tempMatrix[0].at<float>(imagePoints[i].y-2, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x-2) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y-1, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x-1) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y+ imagePoints[i].x+1) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y+1, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y, imagePoints[i].x+2) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					tempMatrix[0].at<float>(imagePoints[i].y+2, imagePoints[i].x) = sqrt(points[i].z*points[i].z+points[i].y*points[i].y+points[i].x*points[i].x);
					//tempMatrix[1].at<float>(imagePoints[i].y-2, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y-1, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y+1, imagePoints[i].x) = atan2(points[i].x, points[i].z);
					//tempMatrix[1].at<float>(imagePoints[i].y+2, imagePoints[i].x) = atan2(points[i].x, points[i].z);
				}
			}
			cv::flip(tempMatrix[0], tempMatrix[0], 1);
			//cv::flip(tempMatrix[1], tempMatrix[1], 1);
			Mat dilateElement = getStructuringElement(MORPH_RECT, Size(6, 14));
			Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
			
			//dilate(tempMatrix[0], tempMatrix[0], dilateElement);
			//erode(tempMatrix[0], tempMatrix[0], erodeElement);
			
			//merge(tempMatrix, 3, newMatrix);
			distanceMat = tempMatrix[0];
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
	ros::Rate loop_rate(4);

	Mat cameraFeed;
	vector<shared_ptr<ofstream> > dataFile;
	

	vector<Mat> distROI;

	vector<Mat> hsvROI;
	Mat hsvROIChannels[3];

	vector<Mat> yCrCbROI;
	Mat yCrCbROIChannels[3];

	vector<Mat> grayscaleROI;
	Mat grayscaleROIChannels[3];
	Mat grayscaleROITemp;

	vector<Mat> averageROI;
	Mat averageROIChannels[3];

	vector<Mat> rgbROI;
	Mat rgbROIChannels[3];

	vector<Mat> greyEdgeROI;
	Mat greyEdgeROIChannels[3];

	VideoWriter vidwriter("cameraFeed.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(640, 480), true);
    	while(1)
	{
		while(!cameraFeed.rows)
		{
			cameraFeed = ic->cameraFeedRos;
			ros::spinOnce();
		}
		blur(cameraFeed, cameraFeed, Size(12,12));
		cvSetMouseCallback("cameraFeed", mouseHandler, NULL);
        	namedWindow("cameraFeed", CV_WINDOW_AUTOSIZE);
        	namedWindow("lidar", CV_WINDOW_AUTOSIZE);

		distROI.clear();
		hsvROI.clear();
		yCrCbROI.clear();
		grayscaleROI.clear();
		averageROI.clear();
		rgbROI.clear();
		greyEdgeROI.clear();
		objVec.clear();

		trackObject(cameraFeed, distanceMat);

		//now that we have a vector of objects, objVec, we can make a bounding box around each detected object.
		//for each object in objVec create a box
		//for(int i=0; i<objVec.size(); i++)
		//{
			//rect.push_back(Rect(objVec[i].xPos-10, objVec[i].yPos-10, objVec[i].xPos+10, objVec[i].yPos+10));
		//}
		//numSamples=objVec.size();
		if(theFlag){
			deltaY=point2.y-point1.y;
			deltaX=point2.x-point1.x;
			numPixels.push_back(deltaY*deltaX);
			stringstream streamer;
			streamer<<numSamples;
			string numSamplesStringer;
			numSamplesStringer=streamer.str();
			string name="/home/travis/Documents/experiment/dataFile"+numSamplesStringer+".txt";
			dataFile.push_back(make_shared<ofstream>(name.c_str()));
			*dataFile[numSamples] << "distance\thChannelTemp\tsChannelTemp\tvChannelTemp\tYChannelTemp\tCrChannelTemp\tCbChannelTemp\tgrayChannelTemp\t";
			*dataFile[numSamples] << "averageColorChannelTemp\tbChannelTemp\tgChannelTemp\trChannelTemp\tbGreyChannelTemp\tgGreyChannelTemp\trGreyChannelTemp\n";
			theFlag=false;
		}	
		if(boxCounter){
			
			cout << "Start" << endl;
			for(int i=0; i<=numSamples; i++){
				stringstream stream;
				stream<<i;
				string numSamplesString;
				numSamplesString=stream.str();
				
			    	//namedWindow("distROI"+numSamplesString, CV_WINDOW_AUTOSIZE);
			    	//namedWindow("hsvROI"+numSamplesString, CV_WINDOW_AUTOSIZE);
			    	//namedWindow("yCrCbROI"+numSamplesString, CV_WINDOW_AUTOSIZE);
			    	//namedWindow("grayscaleROI"+numSamplesString, CV_WINDOW_AUTOSIZE);
			    	//namedWindow("averageROI"+numSamplesString, CV_WINDOW_AUTOSIZE);
			    	//namedWindow("rgbROI"+numSamplesString, CV_WINDOW_AUTOSIZE);
			    	//namedWindow("greyEdgeROI"+numSamplesString, CV_WINDOW_AUTOSIZE);

				//cout << "hsvROi.size() is:" << hsvROI.size() << endl;
				//cout << "rect[numSamples] is :" << rect[numSamples] << endl;
				//cout << "numSamples is :" << numSamples << endl;
				//cout << "numSamplesString is :" << numSamplesString << endl << endl;
				
				distROI.push_back(distanceMat(rect[i]).clone());
				
				float theSum=0;
				int theNumPixels=0;
				for (int j = 0; j<distROI[i].cols; j++)
				{
					for(int k=0; k<distROI[i].rows; k++)
					{
						if(distROI[i].at<float>(j, k) >= 1||distROI[i].at<float>(j, k) <= 3);
						{
							//cout << distROI[i].at<float>(j, k) << endl;
							theSum+=distROI[i].at<float>(j, k);
							//cout << theSum << endl;
							theNumPixels++;
							//cout << theNumPixels << endl;
						}
					}
				}

				cout<< "the distance is:" << theSum/theNumPixels << endl;
	
				hsvROI.push_back(cameraFeed(rect[i]).clone());
				yCrCbROI.push_back(cameraFeed(rect[i]).clone());
				grayscaleROI.push_back(cameraFeed(rect[i]).clone());
				averageROI.push_back(cameraFeed(rect[i]).clone());
				rgbROI.push_back(cameraFeed(rect[i]).clone());
				greyEdgeROI.push_back(cameraFeed(rect[i]).clone());

			    	cvtColor(hsvROI[i], hsvROI[i], COLOR_BGR2HSV);
			    	cvtColor(yCrCbROI[i], yCrCbROI[i], COLOR_BGR2YCrCb);
			    	cvtColor(grayscaleROI[i], grayscaleROI[i], COLOR_BGR2GRAY);
				greyEdgeROI[i]=greyEdge(greyEdgeROI[i]);

				split(hsvROI[i], hsvROIChannels);
				//imshow("channels0", hsvROIChannels[0]);
				//imshow("channels1", hsvROIChannels[1]);
				//imshow("channels2", hsvROIChannels[2]);

				split(yCrCbROI[i], yCrCbROIChannels);
				//imshow("channels00", yCrCbROIChannels[0]);
				//imshow("channels01", yCrCbROIChannels[1]);
				//imshow("channels02", yCrCbROIChannels[2]);

				//grayScale needs no split since it is only one channel

				split(averageROI[i], averageROIChannels);
				//imshow("channels10", averageROIChannels[0]);
				//imshow("channels11", averageROIChannels[1]);
				//imshow("channels12", averageROIChannels[2]);

				split(rgbROI[i], rgbROIChannels);
				//imshow("channels20", rgbROIChannels[0]);
				//imshow("channels21", rgbROIChannels[1]);
				//imshow("channels22", rgbROIChannels[2]);

				split(greyEdgeROI[i], greyEdgeROIChannels);
				//imshow("channels30", greyEdgeROIChannels[0]);
				//imshow("channels31", greyEdgeROIChannels[1]);
				//imshow("channels32", greyEdgeROIChannels[2]);

				//Variables that will be written to data file
				//distance
				//h channel
				//v channel
				//s channel
				//Y channel
				//Cr channel
				//Cb channel
				//gray channel
				//average color channel
				//b channel
				//g channel
				//r channel
				//bGE channel
				//gGE channel
				//rGE channel
				//float hChannelTemp, sChannelTemp, vChannelTemp, YChannelTemp, CrChannelTemp, CbChannelTemp;
				//float grayChannelTemp, averageColorChannelTemp, bChannelTemp, gChannelTemp, rChannelTemp;
				//float bGreyChannelTemp, gGreyChannelTemp, rGreyChannelTemp;

				vector<float> valueVector;

				valueVector.push_back(sum(distROI[0])[0]/numPixels[i]);

				valueVector.push_back(sum(hsvROIChannels[0])[0]/numPixels[i]);
				valueVector.push_back(sum(hsvROIChannels[1])[0]/numPixels[i]);
				valueVector.push_back(sum(hsvROIChannels[2])[0]/numPixels[i]);	

				valueVector.push_back(sum(yCrCbROIChannels[0])[0]/numPixels[i]);	
				valueVector.push_back(sum(yCrCbROIChannels[1])[0]/numPixels[i]);
				valueVector.push_back(sum(yCrCbROIChannels[2])[0]/numPixels[i]);	

				valueVector.push_back(sum(grayscaleROI[i])[0]/numPixels[i]);
				
				valueVector.push_back((sum(rgbROIChannels[0])[0]+sum(rgbROIChannels[1])[0]+sum(rgbROIChannels[2])[0])/(3*numPixels[i]));
				
				valueVector.push_back(sum(rgbROIChannels[0])[0]/numPixels[i]);	
				valueVector.push_back(sum(rgbROIChannels[1])[0]/numPixels[i]);
				valueVector.push_back(sum(rgbROIChannels[2])[0]/numPixels[i]);

				valueVector.push_back(sum(greyEdgeROIChannels[0])[0]/numPixels[i]);	
				valueVector.push_back(sum(greyEdgeROIChannels[1])[0]/numPixels[i]);
				valueVector.push_back(sum(greyEdgeROIChannels[2])[0]/numPixels[i]);

				//write data
				for(int w=0; w<valueVector.size(); w++){
					stringstream writer;
					writer<<valueVector[w];
					string valString;
					valString=writer.str();
					if(w==7) *dataFile[i]<<valString+"\t\t\t";
					else *dataFile[i]<<valString+"\t\t";
				}
				*dataFile[i]<<"\n";
				merge(hsvROIChannels, 3, hsvROI[i]);
				merge(yCrCbROIChannels, 3, yCrCbROI[i]);
				merge(averageROIChannels, 3, averageROI[i]);
				merge(rgbROIChannels, 3, rgbROI[i]);
				merge(greyEdgeROIChannels, 3, greyEdgeROI[i]);
				
				//imshow("hsvROI"+numSamplesString, hsvROI[i]);
				//imshow("yCrCbROI"+numSamplesString, yCrCbROI[i]);
				//imshow("grayscaleROI"+numSamplesString, grayscaleROI[i]);
				//imshow("averageROI"+numSamplesString, averageROI[i]);
				//imshow("rgbROI"+numSamplesString, rgbROI[i]);
				//imshow("greyEdgeROI"+numSamplesString, greyEdgeROI[i]);
			}
		}
		vidwriter.write(cameraFeed);
		imshow("cameraFeed", cameraFeed);
		imshow("lidar", distanceMat);
		cameraFeed.rows=0;
		int esc = cvWaitKey(20);
		if( esc == 1048603){
			for(int b=0; b<numSamples; b++){
				dataFile[b]->close();
			}
			break;
		}
		usleep(500000);
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

	for (int y = 0; y < cameraFeed.rows; y++)
	{
		for (int x = 0; x < cameraFeed.cols; x++)

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
	for (int y = 0; y < cameraFeed.rows; y++)
	{
		for (int x = 0; x < cameraFeed.cols; x++)

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

	for (int y = 0; y < cameraFeed.rows; y++)
	{
		for (int x = 0; x < cameraFeed.cols; x++)
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

	for (int y = 0; y < cameraFeed.rows; y++)
	{
		for (int x = 0; x < cameraFeed.cols; x++)
		{
		Point2f location(x, y);
		camChannels2[0].at<uchar>(location.y, location.x)=camChannels2[0].at<uchar>(location.y, location.x)*(illuminationBlue/blueVal);
		//cout << num<< endl;
		}
	}
	for (int y = 0; y < cameraFeed.rows; y++)
	{
		for (int x = 0; x < cameraFeed.cols; x++)

		{
		Point2f location(x, y);
		camChannels2[1].at<uchar>(location.y, location.x)=camChannels2[1].at<uchar>(location.y, location.x)*(illuminationGreen/greenVal);
		//cout << num<< endl;
		}
	}
	for (int y = 0; y < cameraFeed.rows; y++)
	{
		for (int x = 0; x < cameraFeed.cols; x++)
		{
		Point2f location(x, y);
		camChannels2[2].at<uchar>(location.y, location.x)=camChannels2[2].at<uchar>(location.y, location.x)*(illuminationRed/redVal);
		//cout << num<< endl;
		}
	}
	merge(camChannels2, 3, normalized);
	return normalized;
}

void mouseHandler(int event, int x, int y, int flags, void* param)
{
	extern int numSamples;
	extern vector<Rect> rect;
	if (event == CV_EVENT_LBUTTONDOWN && !drag){
		/* left button clicked. ROI selection begins */
		point1 = Point(x, y);
		drag = 1;
	}

	if (event == CV_EVENT_MOUSEMOVE && drag){
		/* mouse dragged. ROI being selected */
		point2 = Point(x, y);
	}

	if (event == CV_EVENT_LBUTTONUP && drag){
		point2 = Point(x, y);
		rect.push_back(Rect(point1.x,point1.y,x-point1.x,y-point1.y));
		drag = 0;
	}

	if (event == CV_EVENT_LBUTTONUP){
		/* ROI selected */
		if(boxCounter) numSamples++;
		boxCounter=true;
		drag = 0;
		theFlag=true;
	}
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

					//cout << location.xPos << ", " << location.yPos << endl;
					//populates the vector for this object with x, y coordinates for plotting
					objVec.push_back(loc);
					drawObject(cameraFeed);
					//cout << objVec.size();
					objectFound = true;
					theFlag=true;
					count++;
				}
				else objectFound = false;
			}

		}
		else putText(cameraFeed, "there are just toooo many objects!", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
	}
}

void drawObject(Mat &cameraFeed)
{
	for (unsigned int i = 0; i<objVec.size(); i++)
	{
		cv::circle(cameraFeed, cv::Point(objVec[i].xPos, objVec[i].yPos), 10, cv::Scalar(0, 0, 0));
		cv::putText(cameraFeed, "object found!", cv::Point(objVec[i].xPos, objVec[i].yPos + 20), 1, 1, Scalar(0, 255, 0));
	}
}
