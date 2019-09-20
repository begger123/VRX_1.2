#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/Float32.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

using namespace boost;
using namespace std;

void *port_pod_func(void *arg);
void *stbd_pod_func(void *arg);

pthread_mutex_t mutex1, mutex2;
pthread_cond_t cond1, cond2;

typedef struct {
	asio::serial_port *port;
} motor_pod_struct;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blm_extra");
    ros::NodeHandle nh;

    pthread_t port_pod_thread;
    pthread_t stbd_pod_thread;

    pthread_mutex_init(&mutex1, NULL);
    pthread_mutex_init(&mutex2, NULL);
    pthread_cond_init(&cond1, NULL);
    pthread_cond_init(&cond2, NULL);

    asio::io_service io1;
    asio::io_service io2;
    asio::serial_port stbd_port(io1);
    asio::serial_port port_port(io2);
    stbd_port.open("/dev/ttyUSB0");
    stbd_port.set_option(asio::serial_port_base::baud_rate(115200));
    port_port.open("/dev/ttyUSB1");
    port_port.set_option(asio::serial_port_base::baud_rate(115200));

    motor_pod_struct port_pod;
    motor_pod_struct stbd_pod;

    port_pod.port = &port_port;
    stbd_pod.port = &stbd_port;

    if(pthread_create(&port_pod_thread, NULL, port_pod_func, (void*) &port_pod)){
		cout << "Thread not started" << endl;
		return 1;
	}
    if(pthread_create(&stbd_pod_thread, NULL, stbd_pod_func, (void*) &stbd_pod)){
		cout << "Thread not started" << endl;
		return 1;
	}

	ros::Rate r(1);
	while(ros::ok()){
		port_port.write_some(asio::buffer("?V 2\n"));
		pthread_mutex_lock(&mutex1);
		pthread_cond_signal(&cond1);
		pthread_mutex_unlock(&mutex1);
		stbd_port.write_some(asio::buffer("?V 2\n"));
		pthread_mutex_lock(&mutex2);
		pthread_cond_signal(&cond2);
		pthread_mutex_unlock(&mutex2);
		r.sleep();
	}

	return 0;
}


void *port_pod_func(void *arg)
{
	ros::NodeHandle nh;
	ros::Publisher portPub = nh.advertise<std_msgs::Float32>("port_pod_volts",1);

	asio::serial_port *port = ((motor_pod_struct*)arg)->port;

	std_msgs::Float32 msg;

	while(1)
	{
		char ret[10] = {0};
		pthread_mutex_lock(&mutex1);
		pthread_cond_wait(&cond1, &mutex1);
		pthread_mutex_unlock(&mutex1);
		port->read_some(asio::buffer(ret, 10));
		if(!strncmp(ret, "?V", 2)){
			msg.data = atoi(ret+6)/10.0;
			portPub.publish(msg);
		}

	}

}
void *stbd_pod_func(void *arg)
{
	ros::NodeHandle nh;
	ros::Publisher stbdPub = nh.advertise<std_msgs::Float32>("stbd_pod_volts",1);

	asio::serial_port *port = ((motor_pod_struct*)arg)->port;

	std_msgs::Float32 msg;

	while(1)
	{
		char ret[10] = {0};
		pthread_mutex_lock(&mutex2);
		pthread_cond_wait(&cond2, &mutex2);
		pthread_mutex_unlock(&mutex2);
		port->read_some(asio::buffer(ret, 10));
		if(!strncmp(ret, "?V", 2)){
			msg.data = atoi(ret+6)/10.0;
			stbdPub.publish(msg);
		}

	}
}
