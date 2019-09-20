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

	blueA=Point32()
	blueA.x=-90.68
	blueA.y=16.7
	blueA.z=.2
	targetsPointCloud.points.append(blueA)
	blueB=Point32()
	blueB.x=-90.7
	blueB.y=16.71
	blueB.z=.2
	targetsPointCloud.points.append(blueB)

	extra1a=Point32()
	extra1a.x=-86.61
	extra1a.y=5.93
	extra1a.z=.2
	targetsPointCloud.points.append(extra1a)
	extra1b=Point32()
	extra1b.x=-86.62
	extra1b.y=5.96
	extra1b.z=.2
	targetsPointCloud.points.append(extra1b)

	extra3a=Point32()
	extra3a.x=-85.99
	extra3a.y=22.73
	extra3a.z=.2
	targetsPointCloud.points.append(extra3a)
	extra3b=Point32()
	extra3b.x=-86.
	extra3b.y=22.73
	extra3b.z=.2
	targetsPointCloud.points.append(extra3b)

	extra4a=Point32()
	extra4a.x=-79.92
	extra4a.y=16.66
	extra4a.z=.2
	targetsPointCloud.points.append(extra4a)
	extra4b=Point32()
	extra4b.x=-79.93
	extra4b.y=16.64
	extra4b.z=.2
	targetsPointCloud.points.append(extra4b)

	while not rospy.is_shutdown():
		pub_pointcloud.publish(targetsPointCloud)
		rate.sleep()

if __name__ == '__main__':
    try:
        spoofPointCloud()
    except rospy.ROSInterruptException:
        pass