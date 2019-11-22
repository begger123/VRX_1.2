#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from scipy.spatial.distance import pdist
from geometry_msgs.msg import Pose2D
import math

from usv_ahc_py.msg import depth_points as depth_points_msg
from usv_ahc_py.msg import cluster as cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg


import numpy as np

#this package will in fact publish 2 tables - one will be a PointCloud, this can be used for visualizaion in rviz, the other will be a cluster_list, which will be used for autonomy
#the detected objects will come into this package in lidar_nwu frame and will be converted into ned_origin frame
class persistanceTable():
    def __init__(self):
        self.pubPersistentCloud = rospy.Publisher('persistanceCloud', PointCloud, queue_size=1)
        self.pubPersistentClusterList = rospy.Publisher('persistanceClusterList', cluster_list_msg, queue_size=1)
        self.drift=2 #proximity new objects are allowed to be to old objects before position is updated
    
        rospy.init_node('persistence_table', anonymous=False)
        rospy.Subscriber('/clusters', cluster_list_msg, self.clusters_callback)
        rospy.Subscriber('/vehicle_pose', Pose2D, self.pose_callback)
        rospy.Subscriber('/p3d_wamv_ned', Odometry, self.state_callback)
        rospy.on_shutdown(self.shutdownHook)
        self.newPose=False
        self.stateData=Odometry()
        
        self.persistantClusterList=cluster_list_msg()
        self.persistantCloud=PointCloud()
        self.persistantCloud.header.frame_id="ned_origin"
        rospy.spin()
    
    def pose_callback(self,Pose2D):
        self.x=Pose2D.x
        self.y=Pose2D.y
        self.theta=Pose2D.theta
        self.newPose = True

    def state_callback(self,Odometry):
        self.stateData.twist=Odometry.twist


    #The persistance table will be an array of markers.
    #In order to check if a new target needs to be added, we will check to see if
    #the published centroid falls within a certain distance from any current centroids
    #This will consist of two for loops, nested
    #The outer loop will iterate through the persistant table
    #The inner loop will iterate through a new positive detection list
    def clusters_callback(self,clusterListMsg):
        #print abs(self.stateData.twist.twist.angular.z)
          if(self.newPose and abs(self.stateData.twist.twist.angular.z)<0.3):
              for i in range(len(clusterListMsg.cluster_list)):
                  #nice little conversion from nwu to ned that also accounts for the lidars offset from the gps on the vehicle
                  tempX=self.x+(clusterListMsg.cluster_list[i].centroid.x+0.85)*math.cos(self.theta)+clusterListMsg.cluster_list[i].centroid.y*math.sin(self.theta)
                  tempY=self.y+(clusterListMsg.cluster_list[i].centroid.x+0.85)*math.sin(self.theta)-clusterListMsg.cluster_list[i].centroid.y*math.cos(self.theta)
                  clusterListMsg.cluster_list[i].centroid.x=tempX
                  clusterListMsg.cluster_list[i].centroid.y=tempY

                  if(not self.inTable(clusterListMsg.cluster_list[i])):
                      #print "adding"
                      self.addBuoy(clusterListMsg.cluster_list[i])


              self.pubPersistentCloud.publish(self.persistantCloud)
              self.pubPersistentClusterList.publish(self.persistantClusterList)
              self.newPose=False
          else:
              print "angular rate is too large, waiting for a cool down"
    
    
    def addBuoy(self,aCluster):
        print "adding new object"
        aPoint=Point32()
        aPoint.x=aCluster.centroid.x
        aPoint.y=aCluster.centroid.y
        aPoint.z=aCluster.centroid.z
        self.persistantCloud.points.append(aPoint)

        aTempCluster=cluster_msg()
        aTempCluster=aCluster
        self.persistantClusterList.cluster_list.append(aTempCluster)

    def shutdownHook(self):
        print "Shutting down persistence table"
    
    def inTable(self,aClusterList):
        #Look through markerArrayMsg to see if the current object is in the liust
        #Only x and y deviations are used as a filter, since many z centroids will be at the same place
        #print "just entered inTable"
        #print self.markerArrayMsg
        itemInTable=False

        for i in range(len(self.persistantCloud.points)):
            #print "the distance is"
            #print (np.sqrt(np.square(.x-self.markerArrayMsg.markers[i].pose.position.x)+np.square(aClusterList.centroid.y-self.markerArrayMsg.markers[i].pose.position.y)))
            if(np.sqrt(np.square(aClusterList.centroid.x-self.persistantCloud.points[i].x)+np.square(aClusterList.centroid.y-self.persistantCloud.points[i].y))<self.drift):
                itemInTable=True
                #print "updating position"
                self.persistantCloud.points[i].x=(self.persistantCloud.points[i].x+aClusterList.centroid.x)/2
                self.persistantCloud.points[i].y=(self.persistantCloud.points[i].y+aClusterList.centroid.y)/2
                self.persistantCloud.points[i].z=(self.persistantCloud.points[i].z+aClusterList.centroid.z)/2

                self.persistantClusterList.cluster_list[i].centroid.x=self.persistantCloud.points[i].x
                self.persistantClusterList.cluster_list[i].centroid.y=self.persistantCloud.points[i].y
                self.persistantClusterList.cluster_list[i].centroid.z=self.persistantCloud.points[i].z

                self.persistantClusterList.cluster_list[i].raw_cluster=aClusterList.raw_cluster;

        return itemInTable
    
if __name__ == '__main__':
    try:
        theTable=persistanceTable()
    except rospy.ROSInterruptException:
        pass
