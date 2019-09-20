#include<the_planner/missions/follow_the_leader.h>

follow_the_leader::follow_the_leader(ros::NodeHandle &nh) : Mission(nh)
{
	this->task=START;
	this->traj_mission_t.type=100;
	this->follow_the_leader_sub=mission_nh_->subscribe("centroids", 10, &follow_the_leader::follow_the_leader_callback, this);
}

follow_the_leader::~follow_the_leader()
{

}

void follow_the_leader::follow_the_leader_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	this->point_cloud_t.points=msg->points;
	for(int i=0; i<point_cloud_t.points.size(); i++){
		point_cloud_t.points[i].y*=-1;
	}
}

void follow_the_leader::loop()
{
	switch(this->task)
	{	
		//get the centroid
		//plot the points around it
		//repeat 3 times
		case START:
		{
			ROS_INFO("The follow the leader mission has begun, heading to 1st waypoint");
			//go to initial waypoint
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			trajMission.data=	{this->lat, this->lon, 1.5,};
			missionComplete.data=false;
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
			}
			missionComplete.data=false;
			this->set_head(this->heading, 10);
			this->task=FIND_AND_FOLLOW_LEADER;
			break;
		}
		case FIND_AND_FOLLOW_LEADER:
		{
			//get the nearest centroid
			//generate 8 points based on centroid and current vehicle position
			//pass as single mission
			ROS_INFO("Just looking to get the leader");

			while(point_cloud_t.points.size()<1 && ros::ok()) {
				ros::spinOnce();
				ROS_INFO("Spinning while waiting for the leader");
			}

			theLeader=point_cloud_t.points[0];
			float radius=5;
			float quadrantSelector=atan2((theState.pose.y-theLeader.y),(theState.pose.x-theLeader.x));

			//publish this command
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			if(quadrantSelector>0&&quadrantSelector<M_PI/2){
				ROS_INFO("Quadrant 1");
				trajMission.data={theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y-radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector>=M_PI/2&&quadrantSelector<=M_PI){
				ROS_INFO("Quadrant 4");
				trajMission.data={theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y-radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector<0&&quadrantSelector>=-M_PI/2){
				ROS_INFO("Quadrant 2");
				trajMission.data={theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector<-M_PI/2&&quadrantSelector>=-M_PI){
				ROS_INFO("Quadrant 3");
				trajMission.data={theLeader.x-radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			//reorder such that the first point is the closest, and they continue in a CCW fashion
			ROS_INFO("Performing first rotation");
			ros::Rate rate(10);
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
				rate.sleep();
			}
			missionComplete.data=false;
			
			this->task=SECOND_SPIN;
			break;
		}
		case SECOND_SPIN:
		{
			float radius=5.5;
			float quadrantSelector=atan2((theState.pose.y-theLeader.y),(theState.pose.x-theLeader.x));
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			if(quadrantSelector>0&&quadrantSelector<M_PI/2){
				ROS_INFO("Quadrant 1");
				trajMission.data={theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y-radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector>=M_PI/2&&quadrantSelector<=M_PI){
				ROS_INFO("Quadrant 4");
				trajMission.data={theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y-radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector<0&&quadrantSelector>=-M_PI/2){
				ROS_INFO("Quadrant 2");
				trajMission.data={theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector<-M_PI/2&&quadrantSelector>=-M_PI){
				ROS_INFO("Quadrant 3");
				trajMission.data={theLeader.x-radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			missionComplete.data=false;
			ROS_INFO("Performing second rotation");
			ros::Rate rate(10);
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
				rate.sleep();
			}
			missionComplete.data=false;
			this->task=THIRD_SPIN;
			break;
		}
		case THIRD_SPIN:
		{
			ROS_INFO("Performing third rotation");

			float radius=5.25;
			float quadrantSelector=atan2((theState.pose.y-theLeader.y),(theState.pose.x-theLeader.x));
			usv16_msgs::Usv16Mission trajMission;
			trajMission.type=0;
			if(quadrantSelector>0&&quadrantSelector<M_PI/2){
				ROS_INFO("Quadrant 1");
				trajMission.data={theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y-radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector>=M_PI/2&&quadrantSelector<=M_PI){
				ROS_INFO("Quadrant 4");
				trajMission.data={theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y-radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector<0&&quadrantSelector>=-M_PI/2){
				ROS_INFO("Quadrant 2");
				trajMission.data={theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x-radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			else if(quadrantSelector<-M_PI/2&&quadrantSelector>=-M_PI){
				ROS_INFO("Quadrant 3");
				trajMission.data={theLeader.x-radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y+radius*1, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y+radius*sqrt(2)/2, 1,
								  theLeader.x+radius*1, theLeader.y+radius*0, 1.5,
								  theLeader.x+radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,
								  theLeader.x+radius*0, theLeader.y-radius*1, 1.5,
								  theLeader.x-radius*sqrt(2)/2, theLeader.y-radius*sqrt(2)/2, 1,};
			}
			missionComplete.data=false;
			ros::Rate rate(10);
			while(!missionComplete.data&&ros::ok()){
				publish_to_controller_NONBLOCK(trajMission);
				ros::spinOnce();
				rate.sleep();
			}
			missionComplete.data=false;
			this->task=FINISHED;
			break;
		}
		case FINISHED:
		{
			ROS_INFO("Follow the leader Complete");
			finished=true;
			break;
		}
	}
}