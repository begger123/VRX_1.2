#!/usr/bin/env python

import rospy
from time import time
import numpy as np
import matplotlib.path as mpltPath
from matplotlib.path import Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2Functions
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

class lidarGeofence():
    def __init__(self):
       
        #Standard Start
        rospy.init_node('lidar_geofence', anonymous=False)
        
        #setup pubs and subs
        self.pubFencedLidar = rospy.Publisher('lidar_fenced', PointCloud2, queue_size=1)
        rospy.Subscriber('wamv/sensors/lidars/lidar_wamv/points', PointCloud2, self.lidarCallback)
        rospy.Subscriber('/p3d_wamv_ned', Odometry, self.state_callback)
 
        #Set function for when node ends
        rospy.on_shutdown(self.shutdownHook)

        #Define the polygon that corresponds to the beach in NED coordinates, this will be a triangle
        self.verts = [
           (-65., 209.),  # southern most point
           (242., 95.),  # north western point
           (330., 235.),  # north eastern point
           (-65., 209.),  # ignored
        ]
        
        self.codes = [
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.CLOSEPOLY,
        ]
        self.path = mpltPath.Path(self.verts, self.codes)
        
        self.start_time=0 #generic initialization of member data
        self.fencedPC2=PointCloud2()
        self.theState=Odometry()
        rospy.spin()

    #Subscribe to the lidar pointcloud
    def lidarCallback(self, point_cloud_2_data):
        #We start time recording for the loop here, since this is when all commands begin
        self.start_time = time()
        #Store the pc2 in member data 
        self.pc2Data=point_cloud_2_data
        #This puts the pc2 data into a generator object, not entirely sure what that all entails, but it allows us to access the pc2 points individually
        self.points = pc2Functions.read_points(point_cloud_2_data, skip_nans=True, field_names=("x", "y", "z", "intensity", "ring"))

        #Call the nect function
        self.checkPoint()
    
    def state_callback(self,state_t):
        self.theState.header=state_t.header
        self.theState.child_frame_id=state_t.child_frame_id
        self.theState.pose=state_t.pose
        self.theState.twist=state_t.twist
        #print(self.theState)

    #Check to see if pointcloud points are within the geofence
    def checkPoint(self):
        #We make sure to zero out the numpy array every time a new cloud is received
        self.cloud = []
        #We iterate through each point in the generator object
        for p in self.points:
            #We only need to check the x-y location
            #Note that the Lidar Point is in NWU, while the vehicle is in NED
            #When we set up the point to check, we can easily convert it to EFI inertial frame from the Lidar BFF by adding it to the vehicles current position
            pointsToCheck = [self.theState.pose.pose.position.x+p[0],self.theState.pose.pose.position.y-p[1]]
            inside = self.path.contains_point(pointsToCheck)
            print pointsToCheck[0]
            #If the point is not within the geofence, append to to a temporary numpy array
            if (inside == False):
                self.cloud.append([p[0], p[1], p[2], p[3], p[4]])

        #Safety error checking to make sure that create_cloud is not called if there is no data
        if (len(self.cloud) > 0):
            #Create the new pointcloud2 from all of the points that are not within the geofence
            self.fencedPC2=pc2Functions.create_cloud(self.pc2Data.header, self.pc2Data.fields, self.cloud) 
        else:
            print "No Points"

        print "Matplotlib contains_points Elapsed time: " + str(time()-self.start_time)
        #Publish the Fenced PointCloud
        self.pubFencedLidar.publish(self.fencedPC2) 
    
    def shutdownHook(self):
        print "Shutting down lidar geofence"
     
if __name__ == '__main__':
    try:
        lidarGeofence=lidarGeofence()
    except rospy.ROSInterruptException:
        pass
