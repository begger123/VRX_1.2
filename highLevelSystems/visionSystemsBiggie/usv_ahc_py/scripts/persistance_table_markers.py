#!/usr/bin/env python
import rospy
from usv_ahc_py.msg import depth_points as depth_points_msg
from usv_ahc_py.msg import cluster as Cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg
from visualization_msgs.msg import MarkerArray as marker_array_msg
from visualization_msgs.msg import Marker as marker_msg
from scipy.spatial.distance import pdist
from geometry_msgs.msg import Pose2D
import math

import numpy as np

#the detected objects will come into this package in lidar_nwu frame, and will be converted into ned_origin frame here
class persistanceTable():
    def __init__(self):
        self.pubMarkers = rospy.Publisher('markers', marker_array_msg, queue_size=1000)
        self.drift=2 #proximity new objects are allowed to be to old objects before position is updated
        self.markerArrayMsg=marker_array_msg()
        #not sure if this intialization is needed
        self.markerArrayMsg.markers=[]
    
        rospy.init_node('persistence_table', anonymous=False)
        rospy.Subscriber('/positive_detections', cluster_list_msg, self.labeled_clusters_callback)
        rospy.Subscriber('/vehicle_pose', Pose2D, self.pose_callback)
        rospy.on_shutdown(self.shutdownHook)
        self.newPose=False
        rospy.spin()
    
    def pose_callback(self,Pose2D):
        self.x=Pose2D.x
        self.y=Pose2D.y
        self.theta=Pose2D.theta
        self.newPose = True


    #The persistance table will be an array of markers.
    #In order to check if a new target needs to be added, we will check to see if
    #the published centroid falls within a certain distance from any current centroids
    #This will consist of two for loops, nested
    #The outer loop will iterate through the persistant table
    #The inner loop will iterate through a new positive detection list
    def labeled_clusters_callback(self,labeledClusterListMsg):
        
        #print "#######################################################"
        #print "#######################################################"
        #print "############labeledClusterListMsg######################"
        #print "#######################################################"
        #print "#######################################################"
        #print labeledClusterListMsg.cluster_list
        #If the table is empty and there are positive detections, add what is in the positive decections method automatically
        #if((len(self.markerArrayMsg.markers)==0) and (not len(labeledClusterListMsg.cluster_list)==0)):
        #    print("Adding Zero-th Element")
        #    #rospy.sleep(2)
        #    #print len(self.markerArrayMsg.markers)
        #    #print range(len(labeledClusterListMsg.cluster_list))
        #    #print len(labeledClusterListMsg.cluster_list)
        #    for i in range(len(labeledClusterListMsg.cluster_list)):
        #        print i
        #        if(not self.inTable(labeledClusterListMsg.cluster_list[i].centroid)):
        #            self.addBuoy(labeledClusterListMsg.cluster_list[i],i)
        if(self.newPose):
            for i in range(len(labeledClusterListMsg.cluster_list)):
                #print i
                #nice little conversion from nwu to ned that also accounts for the lidars offset from the gps on the vehicle
                tempX=self.x+(labeledClusterListMsg.cluster_list[i].centroid.x+0.85)*math.cos(self.theta)+labeledClusterListMsg.cluster_list[i].centroid.y*math.sin(self.theta)
                tempY=self.y+(labeledClusterListMsg.cluster_list[i].centroid.x+0.85)*math.sin(self.theta)-labeledClusterListMsg.cluster_list[i].centroid.y*math.cos(self.theta)
                labeledClusterListMsg.cluster_list[i].centroid.x=tempX
                labeledClusterListMsg.cluster_list[i].centroid.y=tempY
                labeledClusterListMsg.cluster_list[i].centroid.z=labeledClusterListMsg.cluster_list[i].centroid.z

                if(not self.inTable(labeledClusterListMsg.cluster_list[i].centroid)):
                    print "adding"
                    self.addBuoy(labeledClusterListMsg.cluster_list[i],i)
                #rospy.sleep(2)
                #print markerArrayMsg

            #print(len(self.markerArrayMsg.markers))                
            #print "#######################################################"
            #print "#######################################################"
            #print "#################markerArrayMsg########################"
            #print "#######################################################"
            #print "#######################################################"
            #print self.markerArrayMsg.markers
            #rospy.sleep(2)
            self.pubMarkers.publish(self.markerArrayMsg)
            self.newPose=False
    
    def addBuoy(self,cluster_list_t,i):
        #the markerMsg must be reinitialized every time because otherwise the append
        #to the array command will overwrite the entire array
        self.markerMsg=marker_msg()
        #print("adding buoy")
        #print "#######################################################"
        #print "#######################################################"
        #print "###########cluster_list_centroids######################"
        #print "#######################################################"
        #print "#######################################################"
        #print cluster_list_t.centroid.x
        #print cluster_list_t.centroid.y
        #print cluster_list_t.centroid.z
        self.markerMsg.header.frame_id="ned_origin"
        self.markerMsg.header.stamp = rospy.Time()
        now=rospy.get_rostime()
        self.markerMsg.ns = "can_buoy" + str(i*now.to_nsec())
        self.markerMsg.id = 0
        self.markerMsg.type = 3
        self.markerMsg.action = 0
        self.markerMsg.lifetime = rospy.Duration(0.5)
        self.markerMsg.scale.x=1
        self.markerMsg.scale.y=1
        self.markerMsg.scale.z=1
        self.markerMsg.pose.position.x=cluster_list_t.centroid.x
        self.markerMsg.pose.position.y=cluster_list_t.centroid.y
        self.markerMsg.pose.position.z=cluster_list_t.centroid.z
        self.markerMsg.color.r=0
        self.markerMsg.color.g=1.0
        self.markerMsg.color.b=0
        self.markerMsg.color.a=1
        
        self.markerArrayMsg.markers.append(self.markerMsg)
        #print self.markerArrayMsg.markers
        #rospy.sleep(2)

    def shutdownHook(self):
        print "Shutting down persistence table"
    
    def inTable(self,labeled_centroid_t):
        #Look through markerArrayMsg to see if the current object is in the liust
        #Only x and y deviations are used as a filter, since many z centroids will be at the same place
        print "just entered inTable"
        #print self.markerArrayMsg
        itemInTable=False

        #print range(len(self.markerArrayMsg.markers))
        print len(self.markerArrayMsg.markers)
        for i in range(len(self.markerArrayMsg.markers)):
            #print "the distance is"
            #print (np.sqrt(np.square(labeled_centroid_t.x-self.markerArrayMsg.markers[i].pose.position.x)+np.square(labeled_centroid_t.y-self.markerArrayMsg.markers[i].pose.position.y)))
            #print (np.sqrt(np.square(labeled_centroid_t.x-self.markerArrayMsg.markers[i].pose.position.x)+np.square(labeled_centroid_t.y-self.markerArrayMsg.markers[i].pose.position.y))<self.drift)
            if(np.sqrt(np.square(labeled_centroid_t.x-self.markerArrayMsg.markers[i].pose.position.x)+np.square(labeled_centroid_t.y-self.markerArrayMsg.markers[i].pose.position.y))<self.drift):
                itemInTable=True
                #print("Updating Position")
                self.markerArrayMsg.markers[i].pose.position.x=labeled_centroid_t.x
                self.markerArrayMsg.markers[i].pose.position.y=labeled_centroid_t.y
                self.markerArrayMsg.markers[i].pose.position.z=labeled_centroid_t.z
        return itemInTable
    
if __name__ == '__main__':
    try:
        theTable=persistanceTable()
    except rospy.ROSInterruptException:
        pass
