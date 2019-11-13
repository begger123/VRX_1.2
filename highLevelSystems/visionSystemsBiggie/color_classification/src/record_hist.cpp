#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <color_classification/color_classification.h>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#define M_PI 3.14159265358979323846  /* pi */

using namespace cv;
using namespace std;
using namespace boost::filesystem;


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

std::string pack_path;
int DISPLAY;
int white_threshold = 40;
int bin_size =256;

void calcHistBGRColor(Mat image, vector<Mat> &out, Mat mask)
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

	calcHist( &hsv[0], 1, 0, mask, h_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &hsv[1], 1, 0, mask, s_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &hsv[2], 1, 0, mask, v_hist, 1, &histSize, &histRange, uniform, accumulate );

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

//saves histogram to yaml files
void saveHist(vector<Mat> red, vector<Mat> green, vector<Mat> blue, vector<Mat> black, ros::NodeHandle nh)
{
    //save mat directly to computer to be loaded pack later
	string path = pack_path + "/config/images/";
	cv::FileStorage file(path + "red_b.hist", cv::FileStorage::WRITE);
	file << "HA" << red[0];
	file = cv::FileStorage(path + "red_g.hist", cv::FileStorage::WRITE);
	file << "HA" << red[1];
	file = cv::FileStorage(path + "red_r.hist", cv::FileStorage::WRITE);
	file << "HA" << red[2];
	file = cv::FileStorage(path + "green_b.hist", cv::FileStorage::WRITE);
	file << "HA" << green[0];
	file = cv::FileStorage(path + "green_g.hist", cv::FileStorage::WRITE);
	file << "HA" << green[1];
	file = cv::FileStorage(path + "green_r.hist", cv::FileStorage::WRITE);
	file << "HA" << green[2];
	file = cv::FileStorage(path + "blue_b.hist", cv::FileStorage::WRITE);
	file << "HA" << blue[0];
	file = cv::FileStorage(path + "blue_g.hist", cv::FileStorage::WRITE);
	file << "HA" << blue[1];
	file = cv::FileStorage(path + "blue_r.hist", cv::FileStorage::WRITE);
	file << "HA" << blue[2];
	file = cv::FileStorage(path + "black_b.hist", cv::FileStorage::WRITE);
	file << "HA" << black[0];
	file = cv::FileStorage(path + "black_g.hist", cv::FileStorage::WRITE);
	file << "HA" << black[1];
	file = cv::FileStorage(path + "black_r.hist", cv::FileStorage::WRITE);
	file << "HA" << black[2];
    
}

//streamline this later
//now is later
int main(int argc, char **argv)
{
    //for reading strings from yaml


    ros::init(argc, argv, "Histogram_Generator");
	ros::NodeHandle nh;

    nh.param("/path", pack_path, pack_path);
    nh.param("/display", DISPLAY, DISPLAY);

    ROS_INFO_STREAM(pack_path);

    int num_test_red;
    int num_test_green;
    int num_test_blue;
	int num_test_black;

    //gets array of colors and stores it in vector
    XmlRpc::XmlRpcValue color_list;
    nh.getParam("/colors", color_list);
    vector<string> colors;
    vector<vector<Mat>> hists;

    //for each color that exists
    for(int i = 0; i < color_list.size(); i++)
    {
        //get name of
        string color = (string)color_list[i];
        ROS_INFO_STREAM("Processing: "<< color);
        

        string path = pack_path + "/data/" + color;

        int count = 0;
        vector<Mat> hist;
        for(auto& file : boost::make_iterator_range(directory_iterator(path), {}))
        {   
            //iterating through each file in the folders
            cout << file << endl;
            Mat image = imread(file.path().string(), CV_LOAD_IMAGE_COLOR);
            cvtColor(image, image, CV_RGB2HSV);
            
            if(count == 0)
            {
                calcHistBGRColor(image, hist);
            }
            else
            {

            }
            
            count++;

        }
        colors.push_back(color);
        hists.push_back(hist);
        ROS_INFO_STREAM("Processed " << count << " " << color << " images");
    }
  


    // nh.param("/num_red", num_test_red, num_test_red);
    // nh.param("/num_green", num_test_green, num_test_green);
    // nh.param("/num_blue", num_test_blue, num_test_blue);
	// nh.param("/num_black", num_test_black, num_test_black);


    // //**PROCESSING RED IMAGES**//
    // //**---------------------**//
    // ROS_WARN("PROCESSING %d RED IMAGES", num_test_red);
    // Mat image_red;
    // std::string path = pack_path + "/Data/red test 1.png";
    // std::string path_partial = pack_path + "/Data/red test ";
    // image_red = imread(path, CV_LOAD_IMAGE_COLOR);
	// cvtColor(image_red, image_red, CV_RGB2HSV);

    // vector<Mat> hist_red = getColor(image_red);

    // for(int i = 2; i < num_test_red+1; i++)
    // {
    //     //gets image from file
    //     std::string path_full = path_partial + patch::to_string(i) + ".png";
    //     Mat image_temp = imread(path_full, CV_LOAD_IMAGE_COLOR);
	// 	cvtColor(image_temp, image_temp, CV_RGB2HSV);
    //     vector<Mat> hist_temp = getColor(image_temp);

    //     for(int i = 0; i < 3; i++)
    //     {
    //         Mat temp;
    //         cv::add(hist_red[i], hist_temp[i], temp);
    //         temp/=2;
    //         hist_red[i] = temp;
    //     }

    // }

    // //have to normalize new graph
    // int hist_w = 512; int hist_h = 500;
	// Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    // normalize(hist_red[0], hist_red[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    // normalize(hist_red[1], hist_red[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    // normalize(hist_red[2], hist_red[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    //**FINISHED RED IMAGES**//
    //**-------------------**//



    // //now to save array to yaml file
    // saveHist(hist_red, hist_green, hist_blue, hist_black, nh);
	// ROS_WARN("FINISHED SAVING MATS");

	return 0;

}
