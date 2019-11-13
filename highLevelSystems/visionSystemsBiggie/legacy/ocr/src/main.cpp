#include <ros/ros.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

using namespace std;
using namespace cv;

int main()
{
	Mat image;
	//Eventually this will be replaced with taking a single frame from a video camera
	//This can be done by setting a flag over ros
	image = imread("/home/ubuntu/Desktop/theA.PNG");

	if (!image.data )
	{
		printf("no image data \n");
		return -1;
	}
	//blur(image, image, Size(10,10));
	// Convert RGB image to grayscale
	Mat im_rgb = image;
	Mat im_ycrcb;
	Mat im_bw;

	cvtColor(im_rgb, im_ycrcb, CV_BGR2YCrCb);

	// Convert to binary
	//these are the values for the thresholding, obtrained by program "Tracker"
	int chan1MinTrackbar = 85, chan1MaxTrackbar = 179, chan2MinTrackbar = 146, chan2MaxTrackbar = 255, chan3MinTrackbar = 0, chan3MaxTrackbar = 255;
	inRange(im_ycrcb, Scalar(chan1MinTrackbar, chan2MinTrackbar, chan3MinTrackbar), Scalar(chan1MaxTrackbar, chan2MaxTrackbar, chan3MaxTrackbar), im_bw);

	//the element chosen here is a 4px by 4px rectangle, it will be used
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(2, 2));

	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(2, 2));

	//this needs to get checkout out
	flip(im_bw, im_bw, 1);

	//enlarges the remaining thresholded image
	//dilate(img_bw, img_bw, dilateElement);

	//shrinks the thresholded image
	//erode(img_bw, img_bw, erodeElement);


	// Invert
	Mat src = im_bw;
	Mat dst;
	bitwise_not ( src, dst );
	Mat im_inv = dst;


	// Save to disk
	imwrite("im_rgb.png",im_rgb);
	imwrite("im_ycrcb.png", im_ycrcb);
	imwrite("im_bw.png",im_bw);
	imwrite("im_inv.png",im_inv);

	// Teseract 
	system("tesseract -psm 4 im_ycrcb.png outputYCRCB -1 eng");
	system("tesseract -psm 4 im_bw.png outputBW -1 eng");
	system("tesseract -psm 4 im_inv.png outputINV -1 eng");

	//The image needs to be sent to the ground station/technical directors
	//The contents ""

   return 0;
}
