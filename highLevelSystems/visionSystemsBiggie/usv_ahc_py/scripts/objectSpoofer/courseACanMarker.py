#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

#dockpoint
#-113.35
#-29.93

def spoofPointCloud():
	rospy.init_node('point_spoof', anonymous=True)
	pub_pointcloud = rospy.Publisher('point_cloud', PointCloud, queue_size=1)
	rate = rospy.Rate(10) # 10hz
	targetsPointCloud=PointCloud()
	targetsPointCloud.header.frame_id="world"

	#extra1a=Point32()
	#extra1a.x=-124.6
	#extra1a.y=14.95
	#extra1a.z=.2
	#targetsPointCloud.points.append(extra1a)
	#extra1b=Point32()
	#extra1b.x=-125
	#extra1b.y=15
	#extra1b.z=.2
	#targetsPointCloud.points.append(extra1b)

	can1=Point32()
	can1.x=-81.15
	can1.y=-9.64
	can1.z=.4
	targetsPointCloud.points.append(can1)
	can2=Point32()
	can2.x=-81.16
	can2.y=9.66
	can2.z=.4
	targetsPointCloud.points.append(can2)

	while not rospy.is_shutdown():
		pub_pointcloud.publish(targetsPointCloud)
		rate.sleep()

if __name__ == '__main__':
    try:
        spoofPointCloud()
    except rospy.ROSInterruptException:
        pass