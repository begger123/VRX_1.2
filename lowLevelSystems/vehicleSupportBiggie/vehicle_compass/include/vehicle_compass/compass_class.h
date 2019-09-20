#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/select.h>
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
 #include "tf/transform_datatypes.h"

class serial_compass
{
public:
	serial_compass(ros::NodeHandle&nh);
	~serial_compass();
	int port_setup();
	bool checksumcheck(char * NMEAstring);
	void parsing_compass(char string[50]);
	int loop();
	float Yaw;
	float Pitch;
	float Roll;


private:
	ros::NodeHandle *compass_nh_;
	ros::Publisher compass_pub;
	ros::Time timeNow;
	
	geometry_msgs::Quaternion orientation;

	bool compassFlag;

	//Initialize Serial Port
	char *OSC_FILE_DES = "/dev/ttyUSB1";//<---- The tttsn value of your serial port may change depending on which com port the compass is connected to
	char *OSC_BAUD_RATE = "B38400"; //<----default baudrate of 19200

	//File descriptors.
	int cmp_fd;
    	struct termios old_cmp_tio, new_cmp_tio, old_kbd_tio, new_kbd_tio;
    	int need_exit = 0;
    	int cmp_ind = 0;

	//this parses the standard NMEA string coming in from the OS5000 compass
	char letterdol;
	char letterC;
	char letterP;
	char letterR;
	char letterT;
	float Temperature;
	char letteraster;
	int checksum;
	int i;

	//incoming data variables
	char data_str[100];
   	float variableheading;
   	float variable;

};
