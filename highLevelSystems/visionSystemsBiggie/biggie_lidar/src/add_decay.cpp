#include <biggie_lidar/add_decay.h>

///////////////////****************************************////////////////////////
///////////////////****************************************////////////////////////
///////////////////POINTS MUST BE CONVERTED TO GLOBAL FRAME////////////////////////
///////////////////****************************************////////////////////////
///////////////////****************************************////////////////////////

vision::add_decay::add_decay(ros::NodeHandle &nh) : decay_nh(&nh), loop_rate(4) //sets default loop rate
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
	{
   		ros::console::notifyLoggerLevelsChanged();
	}
	ROS_DEBUG("Setup add_decay subs and pubs");
	pc2_sub = decay_nh->subscribe("lidar_wamv/points", 10, &vision::add_decay::point_cloud_callback, this);
	decayed_pc2_pub = decay_nh->advertise<sensor_msgs::PointCloud2>("/full_point_cloud", 10);
	ROS_DEBUG("get_params");
	this->get_params();
	ROS_DEBUG("fill_buffer");
	this->fill_buffer();
}

vision::add_decay::~add_decay()
{

}

void vision::add_decay::get_params()
{
	//This style of obtaining params is used because it resolves the param relative to the namespace of the node
  	ros::param::get("vision/decay_time", decay_time);
}

void vision::add_decay::fill_buffer()
{
	//length of each pc2 is height*row_step - this is needed when we remove elements
	//something like while(!buffer.full()){buffer.add();}
	std::uint32_t newWidth;
	for(int i=0; i<1; i++)
	{
		//add one pc
		while(!newPC2){
			ros::spinOnce();	
		}
		ROS_DEBUG("heigh: %i",super_pc2.height);
		ROS_DEBUG("width: %i",super_pc2.width);
		//we must update the new width of the point cloud so that RVIZ can properly display, among otther things
		//this width is of the actually point cloud, but the data is transported as bytes
		//row step is the length of the data array that contains the point cloud
		//therefore, width*height*point_step = height*row_step
		newWidth+=super_pc2.height*super_pc2.width;
		width_array.push_back(super_pc2.height*super_pc2.row_step);//we setup this array for housekeeping during push and pop
		for(std::uint32_t j=0; j<super_pc2.height*super_pc2.row_step; j++)
		{
			super_array.push_back(super_pc2.data[j]);
			//ROS_DEBUG("New PC2 Added");
		}
		ROS_DEBUG("vector size: %lu",super_array.size());
		newPC2=false;
	}
	std::vector<std::uint8_t> temp(super_array.begin(),super_array.end());
	super_pc2.data=temp;
	super_pc2.width=newWidth;
}

//this function should only be called when a new pointcloud has been registered
void vision::add_decay::pop_and_push()
{
	//remove first pc2
	for(int i=0;i<width_array[0];i++)
	{
		super_array.pop_front();
	}
	//update width
	width_array.pop_front();
	//add new pc2
	for(std::uint32_t j=0; j<super_pc2.height*super_pc2.row_step; j++)
	{
		super_array.push_back(super_pc2.data[j]);
		//ROS_DEBUG("New PC2 Added");
	}
	//update width
	width_array.push_back(super_pc2.height*super_pc2.row_step);
	std::uint32_t newWidth=0;
	for(int i=0; i<1; i++)
	{
		newWidth+=(width_array[i]/32);
	}
	std::vector<std::uint8_t> temp(super_array.begin(),super_array.end());
	super_pc2.data=temp;
	super_pc2.width=newWidth;
	//publish
	decayed_pc2_pub.publish(super_pc2);
}

void vision::add_decay::point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	super_pc2.header=msg->header;
	super_pc2.height=msg->height;
	super_pc2.width=msg->width;
	super_pc2.fields=msg->fields;
	super_pc2.is_bigendian=msg->is_bigendian;
	super_pc2.point_step=msg->point_step;
	super_pc2.row_step=msg->row_step;
	super_pc2.data=msg->data;
	super_pc2.is_dense=msg->is_dense;

	newPC2=true;
}