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

void calcHistColor(Mat image, vector<Mat> &out, Mat mask)
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

void calcHistColor(Mat image, vector<Mat> &out)
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

void saveHist(vector<vector<Mat>> hists, vector<string> name)
{
    string path = pack_path + "/config/images/";
    for(int i = 0; i < hists.size(); i++)
    {
        cv::FileStorage file(path + name[i] + "_h.hist", cv::FileStorage::WRITE);
        file << "data" << hists[i][0];
        file = cv::FileStorage(path + name[i] + "_s.hist", cv::FileStorage::WRITE);
        file << "data" << hists[i][1];
        file = cv::FileStorage(path  + name[i] + "_v.hist", cv::FileStorage::WRITE);
        file << "data" << hists[i][2];
    }
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
        vector<Mat> hist;   //3 histograms for each channel
        for(auto& file : boost::make_iterator_range(directory_iterator(path), {}))
        {   
            //iterating through each file in the folders
            //cout << file << endl;
            Mat image = imread(file.path().string(), CV_LOAD_IMAGE_COLOR);
            cvtColor(image, image, CV_RGB2HSV);
            
            vector<Mat> temp; 

            calcHistColor(image, temp);
            
            if(count == 0)
            {
                hist = temp;
            }
            else
            {
                for(int i = 0; i < 3; i++)
                {
                    Mat tempHist;
                    cv::add(hist[i], temp[i], tempHist);
                    tempHist/=2;
                    hist[i] = tempHist;
                }
            }

            count++;

        }
        
        int hist_w = 512; int hist_h = 500;
	    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
        normalize(hist[0], hist[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
        normalize(hist[1], hist[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
        normalize(hist[2], hist[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

        colors.push_back(color);
        hists.push_back(hist);

        ROS_INFO_STREAM("Processed " << count << " " << color << " images");
    }
  

    // //now to save array to yaml file
    saveHist(hists, colors);
	ROS_WARN("FINISHED SAVING MATS");

	return 0;

}