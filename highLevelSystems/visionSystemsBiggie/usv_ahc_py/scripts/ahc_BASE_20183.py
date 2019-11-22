#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2Functions
from geometry_msgs.msg import Point32
from usv_ahc_py.msg import depth_points as depth_points_msg
from usv_ahc_py.msg import cluster as Cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage, dendrogram
from scipy.spatial.distance import pdist, squareform
from operator import itemgetter
import math

pubClusters = rospy.Publisher('clusters', cluster_list_msg, queue_size=1)
pubCentroids = rospy.Publisher('centroids', PointCloud, queue_size=1)
pubTest = rospy.Publisher('tester', PointCloud, queue_size=1)

theState=Odometry()

####################################################################################
##################################HELPER FUNCTIONS##################################
####################################################################################
def fancy_dendrogram(*args, **kwargs):
    max_d = kwargs.pop('max_d', None)
    if max_d and 'color_threshold' not in kwargs:
        kwargs['color_threshold'] = max_d
    annotate_above = kwargs.pop('annotate_above', 0)

    ddata = dendrogram(*args, **kwargs)

    if not kwargs.get('no_plot', False):
        plt.title('Hierarchical Clustering Dendrogram (truncated)')
        plt.xlabel('sample index or (cluster size)')
        plt.ylabel('distance')
        for i, d, c in zip(ddata['icoord'], ddata['dcoord'], ddata['color_list']):
            x = 0.5 * sum(i[1:3])
            y = d[1]
            if y > annotate_above:
                plt.plot(x, y, 'o', c=c)
                plt.annotate("%.3g" % y, (x, y), xytext=(0, -5),
                             textcoords='offset points',
                             va='top', ha='center')
        if max_d:
            plt.axhline(y=max_d, c='k')
    return ddata

def state_callback(state_t):
    theState.header=state_t.header
    theState.child_frame_id=state_t.child_frame_id
    theState.pose=state_t.pose
    theState.twist=state_t.twist
    #print(theState)

def shutdownHook():
    print "Shutting down AHC"


####################################################################################
##################################Logical FUNCTIONS#################################
####################################################################################
##Calculates the centroid of the x, y, z elements in a numpy array
def centroidnp(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    sum_z = np.sum(arr[:, 2])
    centroid_temp=Cluster_msg()
    centroid_temp.centroid.x=sum_x/length
    centroid_temp.centroid.y=sum_y/length
    centroid_temp.centroid.z=sum_z/length
    return centroid_temp

def clusterPub(clusters_t,point_cloud_t):
    #declare custom message types
    #print("in cluster pub")
    cluster_list_msg_t=cluster_list_msg()

    #for each element in clusters_t, calculate centroid, get num points, and put in to cluster message
    #then put all of the cluster msgs into a clusters message
    #print(clusters_t)
    labeledPoints=np.insert(point_cloud_t, 3, clusters_t, axis=1)
    #print(labeledPoints[0,3])
    #print(np.unique(clusters_t))
    
    #NOTE - because we are spliting the array based on where differences occur, we must first sort the array
    #I.E - without sorting, an array points with label order 1, 2, 1 would be split into 3 items, not 2!    
    
    #print(np.diff(labeledPoints[:,3]))
    #sorted returns a list, so we must put that back into an nparray
    labeledPoints=np.asarray(sorted(labeledPoints, key=itemgetter(3)))
    #print(labeledPoints[0])
    
    clusters=np.split(labeledPoints, np.where(np.diff(labeledPoints[:,3]))[0]+1)
    #print(len(clusters))
    #populates the cluster list message to send
    centroids_as_pointcloud_t=PointCloud()
    centroids_as_pointcloud_t.header.frame_id="lidar_nwu"
    for i in range(len(clusters)):
        #each cluster array must be converted into an array of 4d points named depth points
        depth_points_list_msg_temp_t=Cluster_msg()
        for j in range(len(clusters[i])):
            depth_points_msg_t=depth_points_msg()
            depth_points_msg_t.labeled_point=clusters[i][j]
            depth_points_list_msg_temp_t.raw_cluster.append(depth_points_msg_t)

        #this function is called once here and stored instead of calling 3 times to pass each point directly as in num_points
        temp=Cluster_msg()
        temp=centroidnp(clusters[i])
        centroids_as_pointcloud_t.points.append(temp.centroid)
        cluster_msg_t=Cluster_msg()
        cluster_msg_t.raw_cluster=depth_points_list_msg_temp_t.raw_cluster
        cluster_msg_t.num_points=len(clusters[i])
        cluster_msg_t.centroid=temp.centroid
        singleDistance=[]
        singleDistance=np.array([[theState.pose.pose.position.x,theState.pose.pose.position.y],[cluster_msg_t.centroid.x,cluster_msg_t.centroid.y]])
        cluster_msg_t.distance=pdist(singleDistance, metric='euclidean')
        cluster_msg_t.distance=math.sqrt(cluster_msg_t.distance*cluster_msg_t.distance)
        #print(singleDistance)
        #print(cluster_msg_t.distance)
        #print(cluster_msg_t.distance)
        #filters out small objects
        if(cluster_msg_t.num_points>0):
            cluster_list_msg_t.cluster_list.append(cluster_msg_t)
        #print(i)
        #print(cluster_list_msg_t)

    #print(centroids_as_pointcloud_t.points)
    #debug printouts
    #print(type(cluster_msg_t.raw_cluster))
    #print(type(cluster_msg_t.num_points))
    #print(type(cluster_msg_t.centroid))
    #print(type(cluster_list_msg_t))
    
    #sorted returns a list, so we must put that back into an nparray
    preSortedCluster=[]
    centroids_as_pointcloud_t=PointCloud()
    centroids_as_pointcloud_t.header.frame_id="lidar_nwu"
    for i in range(len(cluster_list_msg_t.cluster_list)):
        preSortedCluster.append([cluster_list_msg_t.cluster_list[i].raw_cluster, cluster_list_msg_t.cluster_list[i].num_points, 
            cluster_list_msg_t.cluster_list[i].centroid, cluster_list_msg_t.cluster_list[i].label, cluster_list_msg_t.cluster_list[i].distance])

    sortedCluster=np.asarray(sorted(preSortedCluster, key=itemgetter(4)))
    #print("new sorted cluster")
    for i in range(len(sortedCluster)):
        cluster_msg_t=Cluster_msg()
        cluster_msg_t.raw_cluster=sortedCluster[i][0]
        cluster_msg_t.num_points=sortedCluster[i][1]
        cluster_msg_t.centroid=sortedCluster[i][2]
        cluster_msg_t.label=sortedCluster[i][3]
        cluster_msg_t.distance=sortedCluster[i][4]
        #print(cluster_msg_t.distance)
        cluster_list_msg_t.cluster_list.append(cluster_msg_t)

        centroids_as_pointcloud_t.points.append(sortedCluster[i][2])

    #print("clusters")
    #print(centroids_as_pointcloud_t.points)
    #then publish
    #print(cluster_list_msg_t)
    pubClusters.publish(cluster_list_msg_t)

    #go through each cluster
    #calculate the distance from cluster to ship
    #sort cluster message based on the distance
    pubCentroids.publish(centroids_as_pointcloud_t)

def cloudCallback(point_cloud_2):
    point_cloud=PointCloud()
    np_point_cloud=[]
    #the format coming into this function is a sensor_msgs::PointCloud2
    #the data in this pc2 needs to be decoded in order to get x,y,z points
    gen = pc2Functions.read_points(point_cloud_2, skip_nans=True, field_names=("x", "y", "z", "intensity", "ring"))
    #now that we have the x,y,z points, lets turn it into our numpy array


    testingPC=PointCloud()
    testingPC.header.frame_id="lidar_nwu"
    
    for p in gen:
        #only take elements above the water plane
        #this number is 1.5, and not 0, because the points are in the lidar frame of reference, which is 1.5 meters above the water surface
        if(math.sqrt(p[0]*p[0]+p[1]*p[1])> 2 and math.sqrt(p[0]*p[0]+p[1]*p[1])< 25 and p[2]>-1.3):
            np_point_cloud.append([p[0], p[1], p[2]])
            tempPoint=Point32()
            tempPoint.x=p[0]
            tempPoint.y=p[1]
            tempPoint.z=p[2]
            testingPC.points.append(tempPoint)
            #print p

    #this publisher was used to publish to rviz to visually validate that the data is doing what we want 
    #pubTest.publish(testingPC)
 
    thresh=1.25
    if(not np.size(np_point_cloud)==0):
        Y=pdist(np_point_cloud, metric='euclidean')
        Z=linkage(Y, method='average')
        clusterPub(fcluster(Z, thresh, criterion='distance'),np_point_cloud)
        #plt.figure(figsize=(25, 10))
        #plt.title('Hierarchical Clustering Dendrogram')
        #plt.xlabel('sample index')
        #plt.ylabel('distance')
        #dendrogram(
        #    Z,
        #    leaf_rotation=90.,  # rotates the x axis labels
        #    leaf_font_size=8.,  # font size for the x axis labels
        #)
        #plt.show()

        #clusterPub(hcluster.fclusterdata(np_point_cloud, thresh, criterion='distance', method='weighted'),np_point_cloud)
        #clusterPub(hcluster.fclusterdata(np_point_cloud, thresh, criterion='distance', method='centroid'),np_point_cloud)
        #clusterPub(hcluster.fclusterdata(np_point_cloud, thresh, criterion='distance', method='median'),np_point_cloud)
        #clusterPub(hcluster.fclusterdata(np_point_cloud, thresh, criterion='distance', method='ward'),np_point_cloud)

def cloudSub():
    rospy.init_node('cloud_sub', anonymous=False)
    rospy.Subscriber('/lidar_wamv/points', PointCloud2, cloudCallback)
    #rospy.Subscriber('/full_point_cloud', PointCloud2, cloudCallback)
    rospy.Subscriber('/p3d_wamv_ned', Odometry, state_callback)
    rospy.on_shutdown(shutdownHook)
    rospy.spin()


if __name__ == '__main__':
    try:
        cloudSub()
    except rospy.ROSInterruptException:
        pass
