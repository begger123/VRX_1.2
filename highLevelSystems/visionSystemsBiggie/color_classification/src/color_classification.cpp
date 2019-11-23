#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <color_classification/color_classification.h>
#include <string>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#define M_PI 3.14159265358979323846  

using namespace cv;
using namespace std;

float contrast_value = 1.5;
float brightness_value = -10;
int white_threshold = 40;
std::string pack_path;
int bin_size =256;

vector<string> colors;
vector<vector<Mat>> hists;
XmlRpc::XmlRpcValue color_list;

//generates a histogram for color
void findColor(const Mat image, const Mat mask, vector<Point> contours, vector<Mat> &conf)
{
    Rect _boundingRect = boundingRect( contours );
	
	Mat mask_bin;
	//converts to binary (theshold didn't work)
	inRange(mask, Scalar(50, 50, 50), Scalar(255, 255, 255), mask_bin);
	
	// Seperate BGR
	vector<Mat> bgr;
	split( image(_boundingRect), bgr );

	int histSize = bin_size;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	Mat b_hist, g_hist, r_hist;

	calcHist( &bgr[0], 1, 0, mask_bin(_boundingRect), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[1], 1, 0, mask_bin(_boundingRect), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[2], 1, 0, mask_bin(_boundingRect), r_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	
	conf.push_back(b_hist);
	conf.push_back(g_hist);
	conf.push_back(r_hist);
}

void calcHistBGRColor(Mat image, vector<Mat> &out)
{
	// Seperate HSV
	vector<Mat> hsv;
	split( image, hsv );

	int histSize = bin_size;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	Mat h_hist, s_hist, v_hist;

	calcHist( &hsv[0], 1, 0, Mat(), h_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &hsv[1], 1, 0, Mat(), s_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &hsv[2], 1, 0, Mat(), v_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for H, S and V
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	normalize(h_hist, h_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(s_hist, s_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(v_hist, v_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	
	out.push_back(h_hist);
	out.push_back(s_hist);
    out.push_back(v_hist);
}

//displays histograms in a readable way
Mat getDisplayMat(vector<Mat> hist)
{

	if(hist.size()!=3)
		ROS_WARN("Display Input Mat is not the right size");
			
	int histSize = 256;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat display( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );


	// Draw for each channel
	for( int i = 1; i < 256; i++ )
	{
		line( display, Point( bin_w*(i-1), hist_h - cvRound(hist[0].at<float>(i-1)) ) ,
					Point( bin_w*(i), hist_h - cvRound(hist[0].at<float>(i)) ),
					Scalar( 255, 0, 0), 2, 8, 0  );
		line( display, Point( bin_w*(i-1), hist_h - cvRound(hist[1].at<float>(i-1)) ) ,
					Point( bin_w*(i), hist_h - cvRound(hist[1].at<float>(i)) ),
					Scalar( 0, 255, 0), 2, 8, 0  );
		line( display, Point( bin_w*(i-1), hist_h - cvRound(hist[2].at<float>(i-1)) ) ,
					Point( bin_w*(i), hist_h - cvRound(hist[2].at<float>(i)) ),
					Scalar( 0, 0, 255), 2, 8, 0  );
	}

	return display;

}

//grab data from config images
void loadHists()
{
	ROS_INFO("LOADING HISTOGRAM DATA");
	//each color will have 3 histograms, H, S, V
	for(int i = 0; i < color_list.size(); i++)
    {
		string color = (string)color_list[i];
		vector<Mat> colorIn;

		for(int j = 0; j < 3; j++)	//iterate 3 times for the 3 hists
		{
			string path = pack_path + "/config/images/"; 
			string end;
			if(j == 0)
			{
				end = "_h.hist";
			}
			else if(j == 1)
			{
				end = "_s.hist";
			}
			else if(j == 2)
			{
				end = "_v.hist";
			}
			cv::FileStorage file(path + color + end, cv::FileStorage::READ);
			Mat temp;
			file["data"] >> temp;
			colorIn.push_back(temp);
		}
		hists.push_back(colorIn);
	}
	ROS_INFO("FINISHED LOADING HISTOGRAM DATA");
}

//finds colors using h channel
bool findColor(color_classification::color_classification::Request  &req,
	 color_classification::color_classification::Response &res)
{
	ROS_INFO("ENTERED LOOP");
	Mat img;
	vector<Mat> compare;

	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
	img = img_ptr -> image;

	cvtColor(img, img, CV_RGB2HSV);

	vector<Mat> hist_img;
	calcHistBGRColor(img, hist_img);

	vector<float> conf;

	//each color will have 3 histograms, we are just looking at hue
	for(int i = 0; i < color_list.size(); i++)
    {	
		string color = (string)color_list[i];
		Mat toCompare = hists[i][0];
		//compares hue 
		//would have to iterate 3 times for all channels
		conf.push_back(compareHist( hist_img[0], toCompare, CV_COMP_INTERSECT ) );
	}
	
	res.colors = colors;
	res.confidence = conf;

} 

int main(int argc, char **argv)
{

	ros::init(argc, argv, "color_classification_service");
	ros::NodeHandle nh;

	nh.param("/path", pack_path, pack_path);
    nh.getParam("/colors", color_list);
	for(int i = 0; i < color_list.size(); i++)
		colors.push_back((string)color_list[i]);

	loadHists();

	ros::ServiceServer service = nh.advertiseService("color_classification", findColor);
	ROS_INFO("Send image and will return the color of the object");

	ros::spin();

	return 0;

}