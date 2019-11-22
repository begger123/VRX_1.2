#!/usr/bin/env python

import rospy
from usv_ahc_py.msg import cluster as cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg

spoofPLCL=cluster_list_msg()

def spoofNavChannel():
    rospy.init_node('nav_spoof', anonymous=True)
    pub_clusterList=rospy.Publisher('persistanceLabeledClusterList',cluster_list_msg,queue_size=1)
    rate = rospy.Rate(10) # 10hz

    aBuoy=cluster_msg()
    aBuoy.centroid.x=88.8
    aBuoy.centroid.y=119.4
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='white_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=98.8
    aBuoy.centroid.y=114.6
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=73.9
    aBuoy.centroid.y=114.2
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=80.7
    aBuoy.centroid.y=102.0
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=57.0
    aBuoy.centroid.y=92.7
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=67.9
    aBuoy.centroid.y=90.5
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=68.2
    aBuoy.centroid.y=64.5
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=73.3
    aBuoy.centroid.y=76.6
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=93.5
    aBuoy.centroid.y=66.5
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=106.8
    aBuoy.centroid.y=73.7
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=111.9
    aBuoy.centroid.y=49.7
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='blue_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=123
    aBuoy.centroid.y=45.1
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    while not rospy.is_shutdown():
            pub_clusterList.publish(spoofPLCL)
            rate.sleep()

if __name__ == '__main__':
    try:
        spoofNavChannel()
    except rospy.ROSInterruptException:
        pass

