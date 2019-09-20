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

	navChan3a=Point32()
	navChan3a.x=-111.51
	navChan3a.y=-10.72
	navChan3a.z=.2
	targetsPointCloud.points.append(navChan3a)
	navChan3b=Point32()
	navChan3b.x=-111.55
	navChan3b.y=-10.75
	navChan3b.z=.2
	targetsPointCloud.points.append(navChan3b)

	navChan4a=Point32()
	navChan4a.x=-106.39
	navChan4a.y=-9.87
	navChan4a.z=.2
	targetsPointCloud.points.append(navChan4a)
	navChan4b=Point32()
	navChan4b.x=-106.5
	navChan4b.y=-9.88
	navChan4b.z=.2
	targetsPointCloud.points.append(navChan4b)

	while not rospy.is_shutdown():
		pub_pointcloud.publish(targetsPointCloud)
		rate.sleep()

if __name__ == '__main__':
    try:
        spoofPointCloud()
    except rospy.ROSInterruptException:
        pass