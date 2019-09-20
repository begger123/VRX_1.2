#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

def spoofPointCloud():
	rospy.init_node('point_spoof', anonymous=True)
	pub_pointcloud = rospy.Publisher('point_cloud', PointCloud, queue_size=1)
	rate = rospy.Rate(10) # 10hz
	targetsPointCloud=PointCloud()
	targetsPointCloud.header.frame_id="world"

	dock1a=Point32()
	dock1a.x=-97.10
	dock1a.y=-2.42
	dock1a.z=.2
	targetsPointCloud.points.append(dock1a)
	dock1b=Point32()
	dock1b.x=-97.11
	dock1b.y=-2.43
	dock1b.z=.2
	targetsPointCloud.points.append(dock1b)

	dock2a=Point32()
	dock2a.x=-95.5
	dock2a.y=-1.97
	dock2a.z=.2
	targetsPointCloud.points.append(dock2a)
	dock2b=Point32()
	dock2b.x=-95.51
	dock2b.y=-1.98
	dock2b.z=.2
	targetsPointCloud.points.append(dock2b)

	dock3a=Point32()
	dock3a.x=-94.19
	dock3a.y=-1.61
	dock3a.z=.2
	targetsPointCloud.points.append(dock3a)
	dock3b=Point32()
	dock3b.x=-94.2
	dock3b.y=-1.62
	dock3b.z=.2
	targetsPointCloud.points.append(dock3b)

	while not rospy.is_shutdown():
		pub_pointcloud.publish(targetsPointCloud)
		rate.sleep()

if __name__ == '__main__':
    try:
        spoofPointCloud()
    except rospy.ROSInterruptException:
        pass