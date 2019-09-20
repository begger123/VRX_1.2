#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <termios.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include "usv16_blm/blm_status.h"

using namespace usv16_blm;
using namespace boost;
using namespace std;

void *blmFunc(void* arg);
void *getLineFunc(void* arg);

struct blmStruct{
	int argc;
	char **argv;
	asio::serial_port *port;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "blm_node");
	ros::NodeHandle nh;
	asio::io_service io;
	asio::serial_port port(io);

	pthread_t thread;
	pthread_t readThread;

	port.open("/dev/ttyTHS1");
	port.set_option(asio::serial_port_base::baud_rate(115200));

	blmStruct us;
	us.argc = argc;
	us.argv = argv;
	us.port = &port;

	if(pthread_create(&thread, NULL, blmFunc, (void*) &us)){
		cout << "Thread not started" << endl;
		return 1;
	}

	if(pthread_create(&readThread, NULL, getLineFunc, (void*) &us)){
		cout << "Thread not started" << endl;
		return 1;
	}
	ros::Rate r(100);
	ros::spin();


	port.close();
	return 0;
}


void *blmFunc(void* arg){
	asio::serial_port *port = ((blmStruct*) arg)->port;


	ros::init(((blmStruct*) arg)->argc, ((blmStruct*) arg)->argv, "blm_node");
	ros::NodeHandle nh;

	ros::Publisher blmPub = nh.advertise<blm_status>("blm_status",1);

	blm_status blms;

	while(1){


		char ret[1000] = {0};
		port->read_some(asio::buffer(ret, 1000));
		ros::Time time = ros::Time::now();

		if(!strncmp(ret, "<BLM", 4))
		{
			stringstream ss( ret );
			string substr;
			getline(ss, substr, ',' );
			getline(ss, substr, ',' );
			if(substr == "1"){ // Packet type is 1
				blms.header.seq++;
				blms.header.stamp.sec = time.sec;
				blms.header.stamp.nsec = time.nsec;

				getline(ss, substr, ',' );
				blms.rc_kill = atoi(substr.c_str());
				getline(ss, substr, ',' );
				blms.rc_radio = atoi(substr.c_str());
				getline(ss, substr, ',' );
				blms.power_good = atoi(substr.c_str());
				getline(ss, substr, ',' );
				blms.bat_v = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.cpu_v = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.cpu0_i = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.cpu1_i = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.modem_v = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.modem_i = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.sense_v = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.lidar_i = atof(substr.c_str());
				getline(ss, substr, ',' );
				blms.net_i = atof(substr.c_str());
				blmPub.publish(blms);
			}
		}
		usleep(100000);
	}
		/*if(strlen(ret) > 0 && strchr(ret, ',') && strchr(ret, ':')){
			cout << "valid" << endl;

			stringstream ss( ret );
			vector<string> result;
			blms.header.seq++;
			ros::Time time = ros::Time::now();
			blms.header.stamp.sec = time.sec;
			blms.header.stamp.nsec = time.nsec;
			while( ss.good() )
			{
				string substr;
				getline( ss, substr, ',' );
				result.push_back( substr );
				string value, key;


				int i;
				if((i = substr.find_first_of(':')) != string::npos){
					key = substr.substr(0,i);
					value = substr.substr(i+1);
					cout << "key " << substr << endl;
					if(key == "freq1"){
						u.frequency1 = atoi(value.c_str());
					} else if(key == "freq2"){
						u.frequency2 = atoi(value.c_str());
					} else if(key == "bearing1"){
						u.bearing1 = atof(value.c_str());
					} else if(key == "bearing2"){
						u.bearing2 = atof(value.c_str());
					} else if(key == "amp1"){
						u.amplitude1 = atof(value.c_str());
					} else if(key == "amp2"){
						u.amplitude2 = atof(value.c_str());
					}
				}


			}

			usblPub.publish(u);
		}
		cout << ret << endl;

	//	cout << ret << endl;
		usleep(100000);
	}*/

}

void *getLineFunc(void* arg)
{
	asio::serial_port *port = ((blmStruct*) arg)->port;

	//if(!port) return NULL;

	while(1){

		string input;

		getline(cin, input);
		input += "\n";
		port->write_some(asio::buffer(input));
	}

}
