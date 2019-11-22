#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <color_classification/color_classification.h>
#include <string>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

#define M_PI 3.14159265358979323846  

using namespace cv;
using namespace std;

float contrast_value = 1.5;
float brightness_value = -10;
int white_threshold = 40;
std::string pack_path;
int bin_size =256;

vector<Mat> red;
vector<Mat> green;
vector<Mat> blue;
vector<Mat> black;

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

//Calculates it for difficult objects but removes white
void calcHistHSVColorMask(Mat image, vector<Mat> &conf)
{
	// Seperate HSV
	vector<Mat> hsv;
	split( image, hsv );

	Mat mask;
	inRange(image, Scalar(0, 0, 200), Scalar(180, 20, 255), mask);
    bitwise_not(mask,mask);

	int histSize = bin_size;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	Mat h_hist, s_hist, v_hist;

	calcHist( &hsv[0], 1, 0, Mat(), h_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &hsv[1], 1, 0, Mat(), s_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &hsv[2], 1, 0, Mat(), v_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	normalize(h_hist, h_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(s_hist, s_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(v_hist, v_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	
	conf.push_back(h_hist);
	conf.push_back(s_hist);
	conf.push_back(v_hist);
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
	Mat red_b, red_g, red_r;
	string path = pack_path + "/config/images/";
	cv::FileStorage file(path + "red_b.hist", cv::FileStorage::READ);
	file["HA"] >> red_b;
	red.push_back(red_b);
	file = cv::FileStorage(path + "red_g.hist", cv::FileStorage::READ);
	file["HA"] >> red_g;
	red.push_back(red_g);
	file = cv::FileStorage(path + "red_r.hist", cv::FileStorage::READ);
	file["HA"] >> red_r;
	red.push_back(red_r);
	Mat green_b, green_g, green_r;
	file = cv::FileStorage(path + "green_b.hist", cv::FileStorage::READ);
	file["HA"] >> green_b;
	green.push_back(green_b);
	file = cv::FileStorage(path + "green_g.hist", cv::FileStorage::READ);
	file["HA"] >> green_g;
	green.push_back(green_g);
	file = cv::FileStorage(path + "green_r.hist", cv::FileStorage::READ);
	file["HA"] >> green_r;
	green.push_back(green_r);
	Mat blue_b, blue_g, blue_r;
	file = cv::FileStorage(path + "blue_b.hist", cv::FileStorage::READ);
	file["HA"] >> blue_b;
	blue.push_back(blue_b);
	file = cv::FileStorage(path + "blue_g.hist", cv::FileStorage::READ);
	file["HA"] >> blue_g;
	blue.push_back(blue_g);
	file = cv::FileStorage(path + "blue_r.hist", cv::FileStorage::READ);
	file["HA"] >> blue_r;
	blue.push_back(blue_r);
	Mat black_b, black_g, black_r;
	file = cv::FileStorage(path + "black_b.hist", cv::FileStorage::READ);
	file["HA"] >> black_b;
	black.push_back(black_b);
	file = cv::FileStorage(path + "black_g.hist", cv::FileStorage::READ);
	file["HA"] >> black_g;
	black.push_back(black_g);
	file = cv::FileStorage(path + "black_r.hist", cv::FileStorage::READ);
	file["HA"] >> black_r;
	black.push_back(black_r);
}

//finds colors for docking
bool findColor(color_classification::color_classification::Request  &req,
	 color_classification::color_classification::Response &res)
{
	Mat img;
	vector<Mat> compare;

	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
	img = img_ptr -> image;

	cvtColor(img, img, CV_RGB2HSV);

	vector<Mat> hist_img;
	calcHistHSVColorMask(img, hist_img);

	vector<float> conf;

	//compare blue and normalizes
	float sum_blue = 0;
	vector<float> blue_temp;
	for(int i = 0; i < 3; i++)	//comparing loop for each channel
	{	
		blue_temp.push_back(compareHist( hist_img[i], blue[i], CV_COMP_INTERSECT ) );
		sum_blue += compareHist( hist_img[i], blue[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)	//normalizing and placement loop
	{
		blue_temp[i] /= sum_blue;
	}
	//compare green
	float sum_green = 0;
	vector<float> green_temp;
	for(int i = 0; i < 3; i++)	//comparing loop for each channel
	{
		green_temp.push_back(compareHist( hist_img[i], green[i], CV_COMP_INTERSECT ));
		sum_green += compareHist( hist_img[i], green[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)	//normalizing and placement loop
	{
		green_temp[i] /= sum_green;
	}
	//compare red
	float sum_red = 0;
	vector<float> red_temp;
	for(int i = 0; i < 3; i++)
	{	
		red_temp.push_back(compareHist( hist_img[i], red[i], CV_COMP_INTERSECT ) );
		sum_red += compareHist( hist_img[i], red[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)
	{
		red_temp[i]/=sum_red;
	}
	//compare black
	float sum_black = 0;
	vector<float> black_temp;
	for(int i = 0; i < 3; i++)
	{
		black_temp.push_back(compareHist( hist_img[i], black[i], CV_COMP_INTERSECT ) );
		sum_black+=compareHist( hist_img[i], black[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)
	{
		black_temp[i]/=sum_black;
	}

	//three different channels
	//sends all h, then s, then v
	for(int i = 0; i < 3; i++)
	{
		conf.push_back(blue_temp[i]);
		conf.push_back(green_temp[i]);
		conf.push_back(red_temp[i]);
		conf.push_back(black_temp[i]);
	}

	//redo this later to remove conf variable
	if(conf[0] > conf[1] && conf[0] > conf[2])	
	{
		res.color = 0;
	}
	else if(conf[1] > conf[0] && conf[1] > conf[2])
	{
		res.color = 1;
	}
	else if(conf[2] > conf[0] && conf[2] > conf[1])
	{
		res.color = 2;
	}
	else if(conf[3] > conf[0] && conf[3] > conf[1] && conf[3] > conf[2])
	{
		res.color = 3;
	}
	else{
		res.color = 10;	//shouldn't get here
	}

	res.confidence_blue = blue_temp;
	res.confidence_green = green_temp;
	res.confidence_red = red_temp;
	res.confidence_black = black_temp;

} 

int main(int argc, char **argv)
{

	ros::init(argc, argv, "color_classification_service");
	ros::NodeHandle nh;

	nh.param("/path", pack_path, pack_path);

	loadHists();

	ros::ServiceServer service = nh.advertiseService("color_classification", findColor);
	ROS_INFO("Send image and will return the color of the object");

	ros::spin();

	return 0;

}
