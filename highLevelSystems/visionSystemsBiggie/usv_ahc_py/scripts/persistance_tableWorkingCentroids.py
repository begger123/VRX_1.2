#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from scipy.spatial.distance import pdist
from geometry_msgs.msg import Pose2D
import math

import numpy as np

#the detected objects will come into this package in lidar_nwu frame, and will be converted into ned_origin frame here
class persistanceTable():
    def __init__(self):
        self.pubMarkers = rospy.Publisher('markers', PointCloud, queue_size=1)
        self.drift=2 #proximity new objects are allowed to be to old objects before position is updated
    
        rospy.init_node('persistence_table', anonymous=False)
        rospy.Subscriber('/centroids', PointCloud, self.centroids_callback)
        rospy.Subscriber('/vehicle_pose', Pose2D, self.pose_callback)
        rospy.Subscriber('/p3d_wamv_ned', Odometry, self.state_callback)
        rospy.on_shutdown(self.shutdownHook)
        self.newPose=False
        self.persistantCloud=PointCloud()
        self.stateData=Odometry()
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
    def centroids_callback(self,centroidsMsg):
        #print abs(self.stateData.twist.twist.angular.z)
        if(self.newPose and abs(self.stateData.twist.twist.angular.z)<0.3):
            for i in range(len(centroidsMsg.points)):
                #nice little conversion from nwu to ned that also accounts for the lidars offset from the gps on the vehicle
                tempX=self.x+(centroidsMsg.points[i].x+0.85)*math.cos(self.theta)+centroidsMsg.points[i].y*math.sin(self.theta)
                tempY=self.y+(centroidsMsg.points[i].x+0.85)*math.sin(self.theta)-centroidsMsg.points[i].y*math.cos(self.theta)
                centroidsMsg.points[i].x=tempX
                centroidsMsg.points[i].y=tempY

                if(not self.inTable(centroidsMsg.points[i])):
                    #print "adding"
                    self.addBuoy(centroidsMsg.points[i])


            self.pubMarkers.publish(self.persistantCloud)
            self.newPose=False
        else:
            print "angular rate is too large, waiting for a cool down"
    
    def addBuoy(self,aCentroid):
        print "adding new object"
        self.aPoint=Point32()
        self.aPoint.x=aCentroid.x
        self.aPoint.y=aCentroid.y
        self.aPoint.z=aCentroid.z
        self.persistantCloud.points.append(self.aPoint)

    def shutdownHook(self):
        print "Shutting down persistence table"
    
    def inTable(self,aCentroid):
        #Look through markerArrayMsg to see if the current object is in the liust
        #Only x and y deviations are used as a filter, since many z centroids will be at the same place
        #print "just entered inTable"
        #print self.markerArrayMsg
        itemInTable=False

        for i in range(len(self.persistantCloud.points)):
            #print "the distance is"
            #print (np.sqrt(np.square(.x-self.markerArrayMsg.markers[i].pose.position.x)+np.square(aCentroid.y-self.markerArrayMsg.markers[i].pose.position.y)))
            if(np.sqrt(np.square(aCentroid.x-self.persistantCloud.points[i].x)+np.square(aCentroid.y-self.persistantCloud.points[i].y))<self.drift):
                itemInTable=True
                print "updating position"
                self.persistantCloud.points[i].x=(self.persistantCloud.points[i].x+aCentroid.x)/2
                self.persistantCloud.points[i].y=(self.persistantCloud.points[i].y+aCentroid.y)/2
                self.persistantCloud.points[i].z=(self.persistantCloud.points[i].z+aCentroid.z)/2
        return itemInTable
    
if __name__ == '__main__':
    try:
        theTable=persistanceTable()
    except rospy.ROSInterruptException:
        pass
