#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <color_classification/color_classification.h>
#include <ros/package.h>

using namespace cv;
using namespace std;

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "test_client_for_color_detect");
    ros::NodeHandle nh;

    string pack_path;
    nh.param("/path", pack_path, pack_path);

    int num_test_red;
    int num_test_green;
    int num_test_blue;
    int num_test_black;

    nh.param("/num_red", num_test_red, num_test_red);
    nh.param("/num_green", num_test_green, num_test_green);
    nh.param("/num_blue", num_test_blue, num_test_blue);
    nh.param("/num_black", num_test_black, num_test_black);

    //testing all blue images
    for(int i = 1; i < num_test_blue; i++)
    {
        Mat image;
        std::string path = pack_path + "/Data/LightTower/blue test " + patch::to_string(i) + ".png";

        image = imread(path, CV_LOAD_IMAGE_COLOR);

        cv_bridge::CvImage cv_image;
        cv_image.image =  image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;


        ros::ServiceClient client = nh.serviceClient<color_classification::color_classification>("color_classification");
        color_classification::color_classification srv;
        cv_image.toImageMsg(srv.request.image);
        
        cout << "blue:" ;
        if (client.call(srv))
        {
            for(int i = 0; i < srv.response.confidence_blue.size(); i++)
            {
                if(i == 0)
                    cout << "\tblue_h: ";
                if(i == 1)
                    cout << "\tblue_s: ";
                if(i == 2) 
                    cout << "\tblue_v: ";
                cout <<  (float)srv.response.confidence_blue[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_green.size(); i++)
            {
                if(i == 0)
                    cout << "\tgreen_h: ";
                if(i == 1)
                    cout << "\tgreen_s: ";
                if(i == 2) 
                    cout << "\tgreen_v: ";
                cout <<  (float)srv.response.confidence_green[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_red.size(); i++)
            {
                if(i == 0)
                    cout << "\tred_h: ";
                if(i == 1)
                    cout << "\tred_s: ";
                if(i == 2) 
                    cout << "\tred_v: ";
                cout <<  (float)srv.response.confidence_red[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_black.size(); i++)
            {
                if(i == 0)
                    cout << "\tblack_h: ";
                if(i == 1)
                    cout << "\tblack_s: ";
                if(i == 2) 
                    cout << "\tblack_v: ";
                cout <<  (float)srv.response.confidence_black[i];
            }
            //cout << endl << "\tcolor: " << (int)srv.response.color << endl << endl;
            cout << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return 1;
        }
    }

    //testing all green images
    for(int i = 1; i < num_test_green; i++)
    {
        Mat image;
        std::string path = pack_path + "/Data/LightTower/green test " + patch::to_string(i) + ".png";

        image = imread(path, CV_LOAD_IMAGE_COLOR);

        cv_bridge::CvImage cv_image;
        cv_image.image =  image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;


        ros::ServiceClient client = nh.serviceClient<color_classification::color_classification>("color_classification");
        color_classification::color_classification srv;
        cv_image.toImageMsg(srv.request.image);
        
        cout << "green:";
        if (client.call(srv))
        {
            for(int i = 0; i < srv.response.confidence_blue.size(); i++)
            {
                if(i == 0)
                    cout << "\tblue_h: ";
                if(i == 1)
                    cout << "\tblue_s: ";
                if(i == 2) 
                    cout << "\tblue_v: ";
                cout <<  (float)srv.response.confidence_blue[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_green.size(); i++)
            {
                if(i == 0)
                    cout << "\tgreen_h: ";
                if(i == 1)
                    cout << "\tgreen_s: ";
                if(i == 2) 
                    cout << "\tgreen_v: ";
                cout <<  (float)srv.response.confidence_green[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_red.size(); i++)
            {
                if(i == 0)
                    cout << "\tred_h: ";
                if(i == 1)
                    cout << "\tred_s: ";
                if(i == 2) 
                    cout << "\tred_v: ";
                cout <<  (float)srv.response.confidence_red[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_black.size(); i++)
            {
                if(i == 0)
                    cout << "\tblack_h: ";
                if(i == 1)
                    cout << "\tblack_s: ";
                if(i == 2) 
                    cout << "\tblack_v: ";
                cout <<  (float)srv.response.confidence_black[i];
            }
            //cout << endl << "\tcolor: " << (int)srv.response.color << endl << endl;
            cout << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return 1;
        }
    }


    //testing all red images
    for(int i = 1; i < num_test_red; i++)
    {
        Mat image;
        std::string path = pack_path + "/Data/LightTower/red test " + patch::to_string(i) + ".png";

        image = imread(path, CV_LOAD_IMAGE_COLOR);

        cv_bridge::CvImage cv_image;
        cv_image.image =  image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;


        ros::ServiceClient client = nh.serviceClient<color_classification::color_classification>("color_classification");
        color_classification::color_classification srv;
        cv_image.toImageMsg(srv.request.image);
        
        cout << "red:";
        if (client.call(srv))
        {
            for(int i = 0; i < srv.response.confidence_blue.size(); i++)
            {
                if(i == 0)
                    cout << "\tblue_h: ";
                if(i == 1)
                    cout << "\tblue_s: ";
                if(i == 2) 
                    cout << "\tblue_v: ";
                cout <<  (float)srv.response.confidence_blue[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_green.size(); i++)
            {
                if(i == 0)
                    cout << "\tgreen_h: ";
                if(i == 1)
                    cout << "\tgreen_s: ";
                if(i == 2) 
                    cout << "\tgreen_v: ";
                cout <<  (float)srv.response.confidence_green[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_red.size(); i++)
            {
                if(i == 0)
                    cout << "\tred_h: ";
                if(i == 1)
                    cout << "\tred_s: ";
                if(i == 2) 
                    cout << "\tred_v: ";
                cout <<  (float)srv.response.confidence_red[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_black.size(); i++)
            {
                if(i == 0)
                    cout << "\tblack_h: ";
                if(i == 1)
                    cout << "\tblack_s: ";
                if(i == 2) 
                    cout << "\tblack_v: ";
                cout <<  (float)srv.response.confidence_black[i];
            }
            //cout << endl << "\tcolor: " << (int)srv.response.color << endl << endl;
            cout << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return 1;
        }
    }



    //testing all black images
    for(int i = 1; i < num_test_black; i++)
    {
        Mat image;
        std::string path = pack_path + "/Data/LightTower/blank test " + patch::to_string(i) + ".png";

        image = imread(path, CV_LOAD_IMAGE_COLOR);

        cv_bridge::CvImage cv_image;
        cv_image.image =  image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;


        ros::ServiceClient client = nh.serviceClient<color_classification::color_classification>("color_classification");
        color_classification::color_classification srv;
        cv_image.toImageMsg(srv.request.image);
        
        cout << "black:";
        if (client.call(srv))
        {
            for(int i = 0; i < srv.response.confidence_blue.size(); i++)
            {
                if(i == 0)
                    cout << "\tblue_h: ";
                if(i == 1)
                    cout << "\tblue_s: ";
                if(i == 2) 
                    cout << "\tblue_v: ";
                cout <<  (float)srv.response.confidence_blue[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_green.size(); i++)
            {
                if(i == 0)
                    cout << "\tgreen_h: ";
                if(i == 1)
                    cout << "\tgreen_s: ";
                if(i == 2) 
                    cout << "\tgreen_v: ";
                cout <<  (float)srv.response.confidence_green[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_red.size(); i++)
            {
                if(i == 0)
                    cout << "\tred_h: ";
                if(i == 1)
                    cout << "\tred_s: ";
                if(i == 2) 
                    cout << "\tred_v: ";
                cout <<  (float)srv.response.confidence_red[i];
            }
            //cout << endl;
            for(int i = 0; i < srv.response.confidence_black.size(); i++)
            {
                if(i == 0)
                    cout << "\tblack_h: ";
                if(i == 1)
                    cout << "\tblack_s: ";
                if(i == 2) 
                    cout << "\tblack_v: ";
                cout <<  (float)srv.response.confidence_black[i];
            }
            //cout << endl << "\tcolor: " << (int)srv.response.color << endl << endl;
            cout << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return 1;
        }
    }


    //ros::spin();
  return 0;
}
