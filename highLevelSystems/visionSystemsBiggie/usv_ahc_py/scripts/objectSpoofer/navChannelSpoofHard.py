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
    aBuoy.centroid.x=86.2
    aBuoy.centroid.y=120.4
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='white_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=97.5
    aBuoy.centroid.y=115.2
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=89.7
    aBuoy.centroid.y=97.9
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=98.8
    aBuoy.centroid.y=93.4
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=63.4
    aBuoy.centroid.y=85.7
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=71.6
    aBuoy.centroid.y=84.4
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=63.7
    aBuoy.centroid.y=52.9
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='green_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=67.9
    aBuoy.centroid.y=63.7
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='red_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=88.6
    aBuoy.centroid.y=40.4
    aBuoy.centroid.z=-0.64
    aBuoy.label.data='blue_can'
    spoofPLCL.cluster_list.append(aBuoy)

    aBuoy=cluster_msg()
    aBuoy.centroid.x=81.6
    aBuoy.centroid.y=45.0
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

