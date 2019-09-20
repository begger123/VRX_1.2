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

	speedGates2a=Point32()
	speedGates2a.x=-102.96
	speedGates2a.y=8.47
	speedGates2a.z=.2
	targetsPointCloud.points.append(speedGates2a)
	speedGates2b=Point32()
	speedGates2b.x=-102.99
	speedGates2b.y=7.48
	speedGates2b.z=.2
	targetsPointCloud.points.append(speedGates2b)

	extra2a=Point32()
	extra2a.x=-103.14
	extra2a.y=12.86
	extra2a.z=.2
	targetsPointCloud.points.append(extra2a)
	extra2b=Point32()
	extra2b.x=-103.4
	extra2b.y=12.9
	extra2b.z=.2
	targetsPointCloud.points.append(extra2b)

	while not rospy.is_shutdown():
		pub_pointcloud.publish(targetsPointCloud)
		rate.sleep()

if __name__ == '__main__':
    try:
        spoofPointCloud()
    except rospy.ROSInterruptException:
        pass