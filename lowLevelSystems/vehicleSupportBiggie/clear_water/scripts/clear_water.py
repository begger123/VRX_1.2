#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2Functions
import math
import numpy as np
from sensor_msgs.msg import PointField

pubClearedPC2 = rospy.Publisher('/lidar_wamv/points_no_water', PointCloud2, queue_size=1)

def shutdownHook():
    print "Shutting down clear_water"

def xyz_array_to_pointcloud2(points, stamp=None, frame_id='None'):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.FLOAT32, 1)];
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg 

def cloudCallback(point_cloud_2):

    pc2_list=[]
    
    gen = pc2Functions.read_points(point_cloud_2, skip_nans=True, field_names=("x", "y", "z", "intensity", "ring"))
    for p in gen:
        #only take elements above the water plane
        #this number is 1.5, and not 0, because the points are in the lidar frame of reference, which is 1.5 meters above the water surface
        if(math.sqrt(p[0]*p[0]+p[1]*p[1])> 0 and math.sqrt(p[0]*p[0]+p[1]*p[1])< 100 and p[2]>-1.3):
            pc2_list.append([p[0], p[1], p[2]])

    np_point_cloud=np.asarray(pc2_list)
    clearedPC2=xyz_array_to_pointcloud2(np_point_cloud,point_cloud_2.header.stamp,"lidar_nwu")
    pubClearedPC2.publish(clearedPC2)

def cloudSub():
    rospy.init_node('clear_water', anonymous=False)
    rospy.Subscriber('/lidar_wamv/points', PointCloud2, cloudCallback)
    rospy.loginfo("Clear water node has started")
    rospy.on_shutdown(shutdownHook)
    rospy.spin()


if __name__ == '__main__':
    try:
        cloudSub()
    except rospy.ROSInterruptException:
        pass
