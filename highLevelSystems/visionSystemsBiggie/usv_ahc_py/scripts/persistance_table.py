#!/usr/bin/env python
import rospy
from usv_ahc_py.msg import depth_points as depth_points_msg
from usv_ahc_py.msg import cluster as Cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg
from visualization_msgs.msg import MarkerArray as marker_array_msg
from visualization_msgs.msg import Marker as marker_msg
from scipy.spatial.distance import pdist

import numpy as np

pubMarkers = rospy.Publisher('markers', marker_array_msg, queue_size=1000)

drift=1


marker_array_msg_t=marker_array_msg()
marker_msg_t=marker_msg()

def shutdownHook():
    print "Shutting down persistence table"

def inTable(labeled_centroid_t):
    #Look through marker_array_msg_t to see if the current object is in the liust
    #Only x and y deviations are used as a filter, since many z centroids will be at the same place
    itemInTable=False
    for i in range(len(marker_array_msg_t.markers)):
        if(np.sqrt(np.square(labeled_centroid_t.x-marker_array_msg_t.markers[i].pose.position.x)+np.square(labeled_centroid_t.y-marker_array_msg_t.markers[i].pose.position.y))<drift):
            itemInTable=True
            print("Updating Position")
            marker_array_msg_t.markers[i].pose.position.x=labeled_centroid_t.x
            marker_array_msg_t.markers[i].pose.position.y=labeled_centroid_t.y
            marker_array_msg_t.markers[i].pose.position.z=labeled_centroid_t.z
    return itemInTable

def addBuoy(cluster_list_t):
    print("adding buoy")
    if (cluster_list_t.label.data=="can_buoy"):
        marker_msg_t.header.frame_id="world"
        marker_msg_t.header.stamp = rospy.Time()
        marker_msg_t.ns = "can_buoy"
        marker_msg_t.id = 0
        marker_msg_t.type = 3
        marker_msg_t.action = 0
        marker_msg_t.lifetime = rospy.Duration(0.5)
        marker_msg_t.scale.x=1
        marker_msg_t.scale.y=1
        marker_msg_t.scale.z=1
        marker_msg_t.pose.position.x=cluster_list_t.centroid.x
        marker_msg_t.pose.position.y=cluster_list_t.centroid.y
        marker_msg_t.pose.position.z=cluster_list_t.centroid.z
        marker_msg_t.color.r=0
        marker_msg_t.color.g=1.0
        marker_msg_t.color.b=0
        marker_msg_t.color.a=1
        marker_array_msg_t.markers.append(marker_msg_t)
    
    elif (cluster_list_t.label.data=="large_sphere_buoy"): 
        marker_msg_t.header.frame_id="world"
        marker_msg_t.header.stamp = rospy.Time()
        marker_msg_t.ns = "large_sphere_buoy"
        marker_msg_t.id = 0
        marker_msg_t.type = 2
        marker_msg_t.action = 0
        marker_msg_t.lifetime = rospy.Duration(0.5)
        marker_msg_t.scale.x=1
        marker_msg_t.scale.y=1
        marker_msg_t.scale.z=1
        marker_msg_t.pose.position.x=cluster_list_t.centroid.x
        marker_msg_t.pose.position.y=cluster_list_t.centroid.y
        marker_msg_t.pose.position.z=cluster_list_t.centroid.z
        marker_msg_t.color.r=0
        marker_msg_t.color.g=1.0
        marker_msg_t.color.b=0
        marker_msg_t.color.a=1
        marker_array_msg_t.markers.append(marker_msg_t)
    
    elif (cluster_list_t.label.data=="small_sphere_buoy"):
        marker_msg_t.header.frame_id="world"
        marker_msg_t.header.stamp = rospy.Time()
        marker_msg_t.ns = "small_sphere_buoy"
        marker_msg_t.id = 0
        marker_msg_t.type = 2
        marker_msg_t.action = 0
        marker_msg_t.lifetime = rospy.Duration(0.5)
        marker_msg_t.scale.x=0.5
        marker_msg_t.scale.y=0.5
        marker_msg_t.scale.z=0.5
        marker_msg_t.pose.position.x=cluster_list_t.centroid.x
        marker_msg_t.pose.position.y=cluster_list_t.centroid.y
        marker_msg_t.pose.position.z=cluster_list_t.centroid.z
        marker_msg_t.color.r=0
        marker_msg_t.color.g=1.0
        marker_msg_t.color.b=0
        marker_msg_t.color.a=1
        marker_array_msg_t.markers.append(marker_msg_t)
    
    elif (cluster_list_t.label.data=="dock"):
        marker_msg_t.header.frame_id="world"
        marker_msg_t.header.stamp = rospy.Time()
        marker_msg_t.ns = "dock"
        marker_msg_t.id = 0
        marker_msg_t.type = 2
        marker_msg_t.action = 0
        marker_msg_t.lifetime = rospy.Duration(0.5)
        marker_msg_t.scale.x=1
        marker_msg_t.scale.y=1
        marker_msg_t.scale.z=1
        marker_msg_t.pose.position.x=cluster_list_t.centroid.x
        marker_msg_t.pose.position.y=cluster_list_t.centroid.y
        marker_msg_t.pose.position.z=cluster_list_t.centroid.z
        marker_msg_t.color.r=0
        marker_msg_t.color.g=1.0
        marker_msg_t.color.b=0
        marker_msg_t.color.a=1
        marker_array_msg_t.markers.append(marker_msg_t)
#The persistance table will be an array of markers.
#In order to check if a new target needs to be added, we will check to see if
#the published centroid falls within a certain distance from any current centroids
#This will consist of two for loops, nested
#The outer loop will iterate through the persistant table
#The inner loop will iterate through a new positive detection list
def labeled_clusters_callback(labeled_cluster_list_msg_t):
    #If the table is empty, add what is in the positive decections method automatically

    marker_array_msg_t.markers=[]
    if((len(marker_array_msg_t.markers)==0) and (not len(labeled_cluster_list_msg_t.cluster_list)==0)):
        #print("Adding Zero-th Element")
        addBuoy(labeled_cluster_list_msg_t.cluster_list[0])

    for i in range(len(labeled_cluster_list_msg_t.cluster_list)):
        if(not inTable(labeled_cluster_list_msg_t.cluster_list[i].centroid)):
            addBuoy(labeled_cluster_list_msg_t.cluster_list[i])

    print(len(marker_array_msg_t.markers))                
    pubMarkers.publish(marker_array_msg_t)

def labeled_clusters_sub():
    rospy.init_node('persistence_table', anonymous=False)
    rospy.Subscriber('/positive_detections', cluster_list_msg, labeled_clusters_callback)
    rospy.on_shutdown(shutdownHook)
    rospy.spin()


if __name__ == '__main__':
    try:
        labeled_clusters_sub()
    except rospy.ROSInterruptException:
        pass