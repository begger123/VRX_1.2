#!/usr/bin/env python

import rospy
from usv_ahc_py.msg import cluster as cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg

spoofPLCL=cluster_list_msg()

def spoofNavChannel():
    rospy.init_node('nav_spoof', anonymous=True)
    pub_clusterList=rospy.Publisher('persistanceLabeledClusterList',cluster_list_msg,queue_size=1)
    rate = rospy.Rate(10) # 10hz

    greenMiddle=cluster_msg()
    greenMiddle.centroid.x=105.3
    greenMiddle.centroid.y=68.0
    greenMiddle.centroid.z=-0.64
    greenMiddle.label.data='green_can'
    spoofPLCL.cluster_list.append(greenMiddle)

    redMiddle=cluster_msg()
    redMiddle.centroid.x=100.6
    redMiddle.centroid.y=79.3
    redMiddle.centroid.z=-0.5
    redMiddle.label.data='red_can'
    spoofPLCL.cluster_list.append(redMiddle)

    whiteStart=cluster_msg()
    whiteStart.centroid.x=123.7
    whiteStart.centroid.y=75.4
    whiteStart.centroid.z=-0.13
    whiteStart.label.data='white_can'
    spoofPLCL.cluster_list.append(whiteStart)

    redStart=cluster_msg()
    redStart.centroid.x=118.9
    redStart.centroid.y=86.6
    redStart.centroid.z=-0.4
    redStart.label.data='red_can'
    spoofPLCL.cluster_list.append(redStart)

    blueEnd=cluster_msg()
    blueEnd.centroid.x=87.0
    blueEnd.centroid.y=60.2
    blueEnd.centroid.z=-1
    blueEnd.label.data='blue_can'
    spoofPLCL.cluster_list.append(blueEnd)

    redEnd=cluster_msg()
    redEnd.centroid.x=82.2
    redEnd.centroid.y=71.2
    redEnd.centroid.z=-1
    redEnd.label.data='red_can'
    spoofPLCL.cluster_list.append(redEnd)

    while not rospy.is_shutdown():
            pub_clusterList.publish(spoofPLCL)
            rate.sleep()

if __name__ == '__main__':
    try:
        spoofNavChannel()
    except rospy.ROSInterruptException:
        pass

