#!/usr/bin/env python
import rospy
from usv_ahc_py.msg import depth_points as depth_points_msg
from usv_ahc_py.msg import cluster as Cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg
from visualization_msgs.msg import MarkerArray as marker_array_msg
from visualization_msgs.msg import Marker as marker_msg
from scipy.spatial.distance import pdist
from planar import BoundingBox
from std_msgs.msg import String

import numpy as np
state_string=String()

pubDetections = rospy.Publisher('positive_detections', cluster_list_msg, queue_size=1)

def shutdownHook():
    print "Shutting down classifier"

def missionCallback(data):
    state_string.data=data.data

def clusters_callback(cluster_list_msg_t):
    delX=[]
    delY=[]
    delZ=[]
    labeled_cluster_list_msg_t=cluster_list_msg()
    #print(cluster_list_msg_t.cluster_list[0].raw_cluster[0])#.labeled_point[3])
    #print(cluster_list_msg_t.cluster_list[0].raw_cluster[1])#.labeled_point[3])
    #print(cluster_list_msg_t.cluster_list)
    #print(len(cluster_list_msg_t.cluster_list))
    for i in range(len(cluster_list_msg_t.cluster_list)):
        xPoints=[]
        yPoints=[]
        zPoints=[]
        for j in range(cluster_list_msg_t.cluster_list[i].num_points):
            xPoints.append(cluster_list_msg_t.cluster_list[i].raw_cluster[j].labeled_point[0])
            yPoints.append(cluster_list_msg_t.cluster_list[i].raw_cluster[j].labeled_point[1])
            zPoints.append(cluster_list_msg_t.cluster_list[i].raw_cluster[j].labeled_point[2])

        delX.append(max(xPoints)-min(xPoints))
        delY.append(max(yPoints)-min(yPoints))
        delZ.append(max(zPoints)-min(zPoints))
        #print(delX)
        #print(delY)
        #print(delZ)
     
    #Nav Channel
    #Speed Gates
    #Autodocking
    #Follow the Leader
    #Buoy Field

    if(state_string.data=="speed_gate_mission"):
        for k in range(len(cluster_list_msg_t.cluster_list)):
            cluster_msg_t=Cluster_msg()
            #large_sphere_buoy
            if((((delZ[k]/(max(delX[k],delY[k])))<2) and ((delZ[k]/(max(delX[k],delY[k])))>1)) and cluster_list_msg_t.cluster_list[k].centroid.z<.5):
                cluster_msg_t.raw_cluster=cluster_list_msg_t.cluster_list[k].raw_cluster
                cluster_msg_t.num_points=cluster_list_msg_t.cluster_list[k].num_points
                cluster_msg_t.centroid=cluster_list_msg_t.cluster_list[k].centroid
                cluster_msg_t.label.data='large_sphere_buoy'
                labeled_cluster_list_msg_t.cluster_list.append(cluster_msg_t)

    if(state_string.data=="nav_channel"):
        for k in range(len(cluster_list_msg_t.cluster_list)):
            cluster_msg_t=Cluster_msg()
            #can_buoy
            if(delX[k]<0.5 and delX[k]>0.1 and delY[k]<0.5 and delY[k]>0.1 and delZ[k]<2.0 and delZ[k]>0.5 and cluster_list_msg_t.cluster_list[k].centroid.z<1):# and cluster_list_msg_t.cluster_list[k].centroid.x<10):
                cluster_msg_t.raw_cluster=cluster_list_msg_t.cluster_list[k].raw_cluster
                cluster_msg_t.num_points=cluster_list_msg_t.cluster_list[k].num_points
                cluster_msg_t.centroid=cluster_list_msg_t.cluster_list[k].centroid
                cluster_msg_t.label.data='can_buoy'
                labeled_cluster_list_msg_t.cluster_list.append(cluster_msg_t)

    if(state_string.data=="auto_dock"):
        for k in range(len(cluster_list_msg_t.cluster_list)):
            cluster_msg_t=Cluster_msg()
            #dock
            if(delX[k]<0.5 and delX[k]>0.1 and delY[k]<0.5 and delY[k]>0.1 and delZ[k]<1.2 and cluster_list_msg_t.cluster_list[k].centroid.z>0.75 and cluster_list_msg_t.cluster_list[k].centroid.z<2):
                cluster_msg_t.raw_cluster=cluster_list_msg_t.cluster_list[k].raw_cluster
                cluster_msg_t.num_points=cluster_list_msg_t.cluster_list[k].num_points
                cluster_msg_t.centroid=cluster_list_msg_t.cluster_list[k].centroid
                cluster_msg_t.label.data='dock'
                labeled_cluster_list_msg_t.cluster_list.append(cluster_msg_t)

    if(state_string.data=="follow_the_leader"):
        for k in range(len(cluster_list_msg_t.cluster_list)):
            cluster_msg_t=Cluster_msg()
            #dock
            #if(delX[k]<0.5 and delX[k]>0.1 and delY[k]<0.5 and delY[k]>0.1 and delZ[k]<1.2 and cluster_list_msg_t.cluster_list[k].centroid.z>0.75 and cluster_list_msg_t.cluster_list[k].centroid.z<2):
            #    cluster_msg_t.raw_cluster=cluster_list_msg_t.cluster_list[k].raw_cluster
            #    cluster_msg_t.num_points=cluster_list_msg_t.cluster_list[k].num_points
            #    cluster_msg_t.centroid=cluster_list_msg_t.cluster_list[k].centroid
            #    cluster_msg_t.label.data='dock'
            #    labeled_cluster_list_msg_t.cluster_list.append(cluster_msg_t)

    if(state_string.data=="buoy_field"):
        for k in range(len(cluster_list_msg_t.cluster_list)):
            cluster_msg_t=Cluster_msg()
            #can_buoy
            if(delX[k]<0.5 and delX[k]>0.1 and delY[k]<0.5 and delY[k]>0.1 and delZ[k]<2.0 and delZ[k]>0.5 and cluster_list_msg_t.cluster_list[k].centroid.z<1):# and cluster_list_msg_t.cluster_list[k].centroid.x<10):
                cluster_msg_t.raw_cluster=cluster_list_msg_t.cluster_list[k].raw_cluster
                cluster_msg_t.num_points=cluster_list_msg_t.cluster_list[k].num_points
                cluster_msg_t.centroid=cluster_list_msg_t.cluster_list[k].centroid
                cluster_msg_t.label.data='can_buoy'
                labeled_cluster_list_msg_t.cluster_list.append(cluster_msg_t)
            #small_sphere_buoy
            if(delX[k]<0.5 and delX[k]>0.1 and delY[k]<0.5 and delY[k]>0.1 and delZ[k]<0.6 and delZ[k]>.1 and cluster_list_msg_t.cluster_list[k].centroid.z<1):# and cluster_list_msg_t.cluster_list[k].centroid.x<10):
                cluster_msg_t.raw_cluster=cluster_list_msg_t.cluster_list[k].raw_cluster
                cluster_msg_t.num_points=cluster_list_msg_t.cluster_list[k].num_points
                cluster_msg_t.centroid=cluster_list_msg_t.cluster_list[k].centroid
                cluster_msg_t.label.data='small_sphere_buoy'
                labeled_cluster_list_msg_t.cluster_list.append(cluster_msg_t)


    pubDetections.publish(labeled_cluster_list_msg_t)
    #rate=rospy.Rate(20)
    #rate.sleep()

def clusters_sub():
    rospy.init_node('classifier', anonymous=False)
    rospy.Subscriber('/clusters', cluster_list_msg, clusters_callback)
    rospy.Subscriber('/hlp_out', String, missionCallback)
    rospy.on_shutdown(shutdownHook)
    rospy.spin()


if __name__ == '__main__':
    try:
        clusters_sub()
    except rospy.ROSInterruptException:
        pass