#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from vrx_gazebo.msg import Task


class GetMap:


    def __init__(self):
        # Subscribers
        self.get_map_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.get_costmap_callback)
        self.placard_sub = rospy.Subscriber("/placard", Bool, self.placard_callback)
        self.task_sub = rospy.Subscriber("/vrx/task/info", Task, self.task_callback)
        # Publishers
        self.dock_path_pub = rospy.Publisher("/dock_path", Float32MultiArray, latch=True, queue_size=10)
        self.dock_explore_pub = rospy.Publisher("/dock_explore", Float32MultiArray, latch=True, queue_size=10)
        # opencv-ros bridge
        self.bridge = CvBridge()
        # class variables
        self.side = True    # true: right, false: left
        self.got_placard = False
        self.got_dock = False
        self.explore_pts_pix = np.zeros((4,2), np.int)
        self.explore_pts_ned = np.zeros((4,2), np.int)
        self.placard_mean = np.zeros((2,1), np.int)
        self.theTask=Task()


    def placard_callback(self, msg):
        self.side = msg.data

    def task_callback(self,taskMsg):
        self.theTask.name=taskMsg.name

    def get_costmap_callback(self, msg):
        if(self.theTask.name == 'scan'):
            # convert occupancy grid into numpy array
            height = msg.info.height
            width = msg.info.width
            ogrid = np.array(msg.data, np.float).reshape(height, width)
            ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
            self.ogrid_resolution = msg.info.resolution

            # convert occupancy grid to opencv image
            ogc = np.zeros((height, width))
            for i in range(0, height):
                for j in range(0, width):
                    ogc[i,j] = ogrid[height-1-i, j]

            # display numpy array ogrid using opencv
            #  cv2.imshow("OGrid_CV", ogc)
            #  cv2.waitKey(30)

            # debugging printouts
            #  rospy.loginfo("ogrid shape = [%g, %g]", ogrid.shape[0], ogrid.shape[1])
            #  rospy.loginfo("ogrid origin = [%g, %g]", ogrid_origin[0], ogrid_origin[1])
            #  rospy.loginfo("ogrid resolution = %g", self.ogrid_resolution)

            # convert OGrid to binary image
            bimg = self.convert_to_binary(ogc)
            #  bimg = bimg[:, 0:500]
            bimg = bimg[:, 0:550]

            # compute 2 classes using k-means clustering
            # np.transpose(np,nonzero()): returns the coordinates of associated to non-zero values
            # 1st column = rows, 2nd column = columns -> 0-indexed, origin at top-left corner (0,0)
            data = np.transpose(np.nonzero(bimg))
            # convert to float32 type
            data = np.float32(data)
            #  rospy.loginfo("data shape = [%g, %g]", data.shape[0], data.shape[1])
            # define criteria for k-means function
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            # apply k-means algorithm
            cost, label, center = cv2.kmeans(data, 1, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
            # Now separate the data, Note the flatten()
            A = data[label.ravel()==0].astype(int)
            # B = data[label.ravel()==1].astype(int)

            labels_img = np.zeros((height, width, 3), np.uint8)
            labels_img[A[:,0], A[:,1]] = (255,0,0)      # Blue
            # labels_img[B[:,0], B[:,1]] = (0,255,0)      # Green

            # compute dock-cluster
            cy_min = np.min(center[:,1])    # minimum y-coordinate (columns direction) among clusters locations
            self.dock_center = 0
            self.dock_size = 0
            self.dock_mean = np.zeros((1, 2))
            eigvector = np.zeros((2, 1))
            side = 1
            if (self.side):
                side = 1                    # 1:right, -1:left
            else:
                side = -1
            #  rospy.loginfo("side = %g", self.side)
            if (cy_min < 450):
                # which cluster it is?
                dock_center = np.argmin(center[:,1])
                # how many dock-pixels do we have?
                dock_cluster = data[label.ravel()==dock_center]
                dock_size = np.size(dock_cluster)
                #  rospy.loginfo("Cy_min = %g", np.min(center[:,1]))
                rospy.loginfo("dock_size = %g", dock_size)
                #  rospy.loginfo("dock_center is: %g", dock_center)
                if (dock_size > 3600):
                    # perform PCA analysis
                    eigvector, eigvector2, angle, angle2, dock_mean = self.do_pca(dock_cluster, side)
                    # draw image and show eigenvector
                    vector = (eigvector*100).astype(int) + dock_mean
                    points = np.zeros((4,2), np.int)
                    # compute starting path-point
                    points[0,:] = vector;
                    # compute stop point
                    points[1,:] = (eigvector*60).astype(int) + dock_mean
                    # compute prior sk-point prior to dock_point
                    points[2,:] = (eigvector*50).astype(int) + dock_mean
                    # compute and draw docking_point
                    points[3,:] = (eigvector*20).astype(int) + dock_mean
                    # Draw all the points and convert them to NED coordinates
                    self.points_ned = np.zeros((points.shape[0], 2))
                    self.points_ned = self.convert_to_ned(points.astype(int), height, width)

                    # Drawing stuff onto opencv image
                    # for drawing the line we have to invert the coordinates
                    #  cv2.line(labels_img, (dock_mean[1], dock_mean[0]), (vector[1], vector[0]), (255,255,255), 2)
                    #  for i in range(points.shape[0]):
                    #      cv2.circle(labels_img, (points[i,1], points[i,0]), 4, (255,0,255), -1)

                    # publish via ROS
                    dock_path_ned = Float32MultiArray()
                    for i in range(self.points_ned.shape[0]):
                        dock_path_ned.data.append(self.points_ned[i,0])
                        dock_path_ned.data.append(self.points_ned[i,1])
                    self.dock_path_pub.publish(dock_path_ned)

                    j = 0
                    for i in range(self.points_ned.shape[0]):
                        rospy.loginfo("dock_path_ned = [%g, %g]", dock_path_ned.data[j], dock_path_ned.data[j+1])
                        j = j + 2


                    # debug printouts
                    #  rospy.loginfo("dock_centroid = [%g, %g]", dock_mean[0], dock_mean[1])
                    #  rospy.loginfo("eigvector = [%g, %g]", eigvector[0], eigvector[1])
                    #  rospy.loginfo("angle = %g deg.", angle*180/np.pi)
                    #  rospy.loginfo("ogrid_resolution = %g", self.ogrid_resolution)
                    #  rospy.loginfo("vector = [%g, %g]", vector[0], vector[1])

            # draw all about placard
            #  for i in range(self.explore_pts_pix.shape[0]):
            #      cv2.circle(labels_img, (self.explore_pts_pix[i,1], self.explore_pts_pix[i,0]), 4, (255,0,255), -1)
            #      #  rospy.loginfo("explore_points = [%g, %g]", self.explore_pts_pix[i,0], self.explore_pts_pix[i,1])

            #  cv2.line(labels_img, (self.placard_mean[1], self.placard_mean[0]), (self.vec_placard1[1], self.vec_placard1[0]), (255,255,255), 2)
            #  cv2.line(labels_img, (self.placard_mean[1], self.placard_mean[0]), (self.vec_placard2[1], self.vec_placard2[0]), (255,255,255), 2)

            # draw dock_paths
            # cv2.imshow("Labels Image", labels_img)
            # cv2.waitKey(30)
        else:
            #this will only do if the right mission
            doNothing=1


    def convert_to_ned(self, array, height, width):
        array_ned = np.zeros((array.shape[0], array.shape[1]), np.float)
        for i in range(array.shape[0]):
            xned = (height - 1 - array[i,0])*0.3
            yned = array[i,1]*0.3
            array_ned[i,:] = [xned, yned]
            #  rospy.loginfo("array_ned = [%g, %g]", array_ned[i,0], array_ned[i,1])
        return array_ned


    def convert_to_binary(self, Fimg):
        th = 2
        height = Fimg.shape[0]
        width = Fimg.shape[1]
        Bimg = np.zeros((height, width))
        Bimg = 1.0*(Fimg > th)
        # debugging printouts
        #  rospy.loginfo("[bin_height, bin_width] = [%i, %i]", height, width)
        #  rospy.loginfo("[min, max] = [%g, %g]", np.min(Bimg), np.max(Bimg))

        #  cv2.imshow("Binary OGrid", Bimg)
        #  cv2.waitKey(30)
        return Bimg


    def do_pca(self, cluster, side):
        # perform PCA analysis
        # (1) compute mean for each feature in the sample set
        dock_mean = np.mean(cluster, axis=0)
        # mean normalization: replace each feature by feature - mean
        cluster_norm = cluster - dock_mean
        # (2) data covariance (first we have to transpose the data
        sigma = np.cov(cluster_norm, rowvar=False, bias=True)
        # (3) singular value decomposition
        u, s, vh = np.linalg.svd(sigma)
        eigvector1 = u[:,0]*side
        eigvector2 = u[:,1]*side
        angle1 = np.arctan2(eigvector1[1], eigvector1[0])
        angle2 = np.arctan2(eigvector2[1], eigvector2[0])
        dock_mean = dock_mean.astype(int)
        return eigvector1, eigvector2, angle1, angle2, dock_mean


if __name__ == "__main__":
    try:
        rospy.init_node("get_costmap_node", anonymous=True)
        getMap = GetMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        #  cv2.destroyAllWindows()
        rospy.loginfo("ogrid to cv_image task has been terminated")








