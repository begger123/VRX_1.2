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

	navChan1a=Point32()
	navChan1a.x=-119.49
	navChan1a.y=7.33
	navChan1a.z=.2
	targetsPointCloud.points.append(navChan1a)
	navChan1b=Point32()
	navChan1b.x=-119.5
	navChan1b.y=7.35
	navChan1b.z=.2
	targetsPointCloud.points.append(navChan1b)

	navChan2a=Point32()
	navChan2a.x=-116.01
	navChan2a.y=9.14
	navChan2a.z=.2
	targetsPointCloud.points.append(navChan2a)
	navChan2b=Point32()
	navChan2b.x=-116.05
	navChan2b.y=9.14
	navChan2b.z=.2
	targetsPointCloud.points.append(navChan2b)

	extra2a=Point32()
	extra2a.x=-122.9
	extra2a.y=3.76
	extra2a.z=.2
	targetsPointCloud.points.append(extra2a)
	extra2b=Point32()
	extra2b.x=-123
	extra2b.y=3.78
	extra2b.z=.2
	targetsPointCloud.points.append(extra2b)

	extra2a=Point32()
	extra2a.x=-122.9
	extra2a.y=3.76
	extra2a.z=.2
	targetsPointCloud.points.append(extra2a)
	extra2b=Point32()
	extra2b.x=-123
	extra2b.y=3.78
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