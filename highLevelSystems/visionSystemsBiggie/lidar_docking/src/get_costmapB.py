#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class GetMap:


    def __init__(self):
        # Subscribers
        self.get_map_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.get_costmap_callback)
        self.plackard_sub = rospy.Subscriber("/plackard", Bool, self.plackard_callback)
        # Publishers
        self.dock_path_pub = rospy.Publisher("/dock_path", Float32MultiArray, latch=True, queue_size=10)
        self.dock_around_pub = rospy.Publisher("/dock_explore", Float32MultiArray, latch=True, queue_size=10)
        # opencv-ros bridge
        self.bridge = CvBridge()
        # class variables
        self.side = True    # true: right, false: left
        self.around_pts_pix = np.zeros((4,2), np.int)
        self.around_pts_ned = np.zeros((4,2), np.int)
        self.dock_mean = np.zeros((2,1), np.int)


    def plackard_callback(self, msg):
        self.side = msg.data


    def get_costmap_callback(self, msg):
        #  rospy.loginfo("got new costmap");
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
        bimg = bimg[:, 0:500]

        labels_img = np.zeros((height, width, 3), np.uint8)
        # get dock info
        dock_region = bimg[:, 0:500]
        dock_points = np.transpose(np.nonzero(dock_region)).astype(int)
        rospy.loginfo("number of dock_points = %g", dock_points.shape[0])
        th1 = 2200 
        self.dock_size = dock_points.shape[0]
        if (self.dock_size > th1):
            rospy.loginfo("got the number of dock points required")
            # perform pca over dock and find centroid, eigenvector and orientation
            labels_img[dock_points[:,0], dock_points[:,1]] = (255, 255, 255)
            eigv_dock1, eigv_dock2, dock_angle1, dock_angle2, dock_mean = self.do_pca(dock_points, 1)
            self.dock_mean = dock_mean
            #  angle_dock = -angle_dock*180/np.pi - 90
            dock_angle1 = dock_angle1 - np.pi/2
            # Compute the points around the dock (around_pts)
            l = 40
            s = 30
            around_pts = np.array([ [-s, l], [-s, -l], [s, -l], [s, l] ])
            rot_mat = np.array([ [np.cos(dock_angle1), np.sin(dock_angle1)], [-np.sin(dock_angle1), np.cos(dock_angle1)] ])
            self.around_pts_pix = np.dot(around_pts, rot_mat).astype(int) + self.dock_mean.astype(int)
            # scaled eigenvectors for plotting
            self.vec_dock1 = (eigv_dock1*60).astype(int) + self.dock_mean
            self.vec_dock2 = (eigv_dock2*60).astype(int) + self.dock_mean
            # convert dock around_points to NED
            self.around_pts_ned = self.convert_to_ned(self.around_pts_pix.astype(int), height, width)
            # publish via ROS
            dock_around_ned = Float32MultiArray()
            j = 0
            for i in range(self.around_pts_ned.shape[0]):
                dock_around_ned.data.append(np.int(self.around_pts_ned[i,0]))
                dock_around_ned.data.append(np.int(self.around_pts_ned[i,1]))
            self.dock_around_pub.publish(dock_around_ned)

            #  # Compute the dock_path:
            #  # Draw straight path
            #  vector = (eigv_dock1*100).astype(int) + self.dock_mean
            #  points = np.zeros((4,2), np.int)
            #  # compute starting path-point
            #  points[0,:] = vector;
            #  # compute stop point
            #  points[1,:] = (eigv_dock1*60).astype(int) + self.dock_mean
            #  # compute prior sk-point prior to dock_point
            #  points[2,:] = (eigv_dock1*50).astype(int) + self.dock_mean
            #  # compute and draw docking_point
            #  points[3,:] = (eigv_dock1*20).astype(int) + self.dock_mean
            #  # for drawing the line we have to invert the coordinates
            #  #  cv2.line(labels_img, (self.dock_mean[1], self.dock_mean[0]), (vector[1], vector[0]), (255,255,255), 2)
            #  # Draw all the points and convert them to NED coordinates
            #  self.points_ned = np.zeros((points.shape[0], 2))
            #  self.points_ned = self.convert_to_ned(points.astype(int), height, width)
            #
            #  for i in range(points.shape[0]):
            #      cv2.circle(labels_img, (points[i,1], points[i,0]), 4, (255,0,255), -1)
            #
            #  # publish via ROS
            #  dock_path_ned = Float32MultiArray()
            #  for i in range(self.points_ned.shape[0]):
            #      dock_path_ned.data.append(self.points_ned[i,0])
            #      dock_path_ned.data.append(self.points_ned[i,1])
            #  self.dock_path_pub.publish(dock_path_ned)
            #
            #  j = 0
            #  for i in range(self.points_ned.shape[0]):
            #      rospy.loginfo("dock_path_ned = [%g, %g]", dock_path_ned.data[j], dock_path_ned.data[j+1])
            #      j = j + 2
            #
            #  # debug printouts
            #  #  rospy.loginfo("dock_centroid = [%g, %g]", dock_mean[0], dock_mean[1])
            #  #  rospy.loginfo("eigv_dock1 = [%g, %g]", eigv_dock1[0], eigv_dock1[1])
            #  #  rospy.loginfo("angle = %g deg.", angle*180/np.pi)
            #  #  rospy.loginfo("ogrid_resolution = %g", self.ogrid_resolution)
            #  #  rospy.loginfo("vector = [%g, %g]", vector[0], vector[1])
            #
            #  # Draw all info computed about the dock
            #  for i in range(self.around_pts_pix.shape[0]):
            #      cv2.circle(labels_img, (self.around_pts_pix[i,1], self.around_pts_pix[i,0]), 4, (255,0,255), -1)
            #      #  rospy.loginfo("around_points = [%g, %g]", self.around_pts_pix[i,0], self.around_pts_pix[i,1])
            #  cv2.line(labels_img, (self.dock_mean[1], self.dock_mean[0]), (self.vec_dock1[1], self.vec_dock1[0]), (255,255,255), 2)
            #  cv2.line(labels_img, (self.dock_mean[1], self.dock_mean[0]), (self.vec_dock2[1], self.vec_dock2[0]), (255,255,255), 2)

        #  draw dock_paths
        #  cv2.imshow("Labels Image", labels_img)
        cv2.imshow("Labels Image", bimg)
        cv2.waitKey(30)


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







