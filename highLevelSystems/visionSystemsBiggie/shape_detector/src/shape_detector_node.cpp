#include <iostream>
#include <ros/ros.h>


//the purpose of this script is to subscribe to a vector of vectors of 3d points
//vec<vec<point6d>>
// - points are cartesion xyz coordinates, coupled with rgb values
// - internal vector is a single clustered object
// - external vector is a grouping of clustered objects as returned by k-nearest-neighbors algorithm
//With this data, this script will determine the apropriate shape classification base on a statistical
//analysis of the size of the shape as described by the points

//a secondary algorithm will be to describe the color of the shape based on a stastical analysis of the
//color distribution
int main(int argc, char** argv)
{
	ROS_INFO("Hello World");
	return 0;
}
