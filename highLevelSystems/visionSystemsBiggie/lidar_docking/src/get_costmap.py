#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt


class GetMap:


    def __init__(self):
        # Subscribers
        self.get_map_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.get_costmap_callback)
        #  self.get_map_update_sub = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, self.get_costmapUpdate_callback)
        self.bridge = CvBridge()


    def get_costmap_callback(self, msg):
        # convert occupancy grid into numpy array
        height = msg.info.height
        width = msg.info.width
        ogrid = np.array(msg.data, np.float).reshape(height, width)
        ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.ogrid_resolution = msg.info.resolution

        # debugging printouts
        #  rospy.loginfo("ogrid shape = [%g, %g]", ogrid.shape[0], ogrid.shape[1])
        #  rospy.loginfo("ogrid origin = [%g, %g]", ogrid_origin[0], ogrid_origin[1])
        #  rospy.loginfo("ogrid resolution = %g", self.ogrid_resolution)

        ogc = np.zeros((height, width))

        for i in range(0, height):
            for j in range(0, width):
                ogc[i,j] = ogrid[height-1-i, j]

        # display numpy array ogrid using opencv
        #  cv2.imshow("OGrid_CV", ogc)
        #  cv2.waitKey(30)

        # convert OGrid to binary image
        bimg = self.convert_to_binary(ogc)

        #  # change the plot orientation if need it
        #  bimg_c = np.zeros((height, width))
        #  for i in range(height):
        #      for j in range(width):
        #          bimg_c[j, width-1-i] = bimg[i, j]

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
        cost, label, center = cv2.kmeans(data, 3, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        # Now separate the data, Note the flatten()
        A = data[label.ravel()==0].astype(int)
        B = data[label.ravel()==1].astype(int)
        C = data[label.ravel()==2].astype(int)

        labels_img = np.zeros((height, width, 3), np.uint8)
        labels_img[A[:,0], A[:,1]] = (255,0,0)      # Blue
        labels_img[B[:,0], B[:,1]] = (0,255,0)      # Green
        labels_img[C[:,0], C[:,1]] = (0,0,255)      # Red

        # compute dock-cluster
        cy_min = np.min(center[:,1])    # minimum y-coordinate (columns direction) among clusters locations
        self.dock_center = 0
        self.dock_size = 0
        self.dock_mean = np.zeros((1, 2))
        eigvector = np.zeros((2, 1))
        side = -1                        # 1:right, -1:left
        if (cy_min < 450):
            # which cluster it is?
            dock_center = np.argmin(center[:,1])
            # how many dock-pixels do we have?
            dock_cluster = data[label.ravel()==dock_center]
            dock_size = np.size(dock_cluster)
            rospy.loginfo("Cy_min = %g", np.min(center[:,1]))
            rospy.loginfo("dock_size = %g", dock_size)
            rospy.loginfo("dock_center is: %g", dock_center)
            if (dock_size > 4300):
                #  # perform PCA analysis
                #  # (1) compute mean for each feature in the sample set
                #  dock_mean = np.mean(dock_cluster, axis=0)
                #  # mean normalization: replace each feature by feature - mean
                #  dock_cluster_norm = dock_cluster - dock_mean
                #  # (2) data covariance (first we have to transpose the data
                #  sigma = np.cov(dock_cluster_norm, rowvar=False, bias=True)
                #  # (3) singular value decomposition
                #  u, s, vh = np.linalg.svd(sigma)
                #  eigvector = u[:,0]*side
                #  angle = np.arctan2(eigvector[1], eigvector[0])
                #  dock_mean = dock_mean.astype(int)

                eigvector, angle, dock_mean = self.do_pca(dock_cluster)

                # draw image and show eigenvector
                vector = (eigvector*100).astype(int) + dock_mean
                # compute and draw docking_point
                docking_point = (eigvector*20).astype(int) + dock_mean
                # for drawing the line we have to invert the coordinates
                cv2.line(labels_img, (dock_mean[1], dock_mean[0]), (vector[1], vector[0]), (255,255,255), 2)
                cv2.circle(labels_img, (docking_point[1], docking_point[0]), 6, (255,0,255), -1)
                cv2.imshow("Labels Image", labels_img)
                cv2.waitKey(30)

                # debug printouts
                rospy.loginfo("dock_centroid = [%g, %g]", dock_mean[0], dock_mean[1])
                rospy.loginfo("eigvector = [%g, %g]", eigvector[0], eigvector[1])
                rospy.loginfo("angle = %g deg.", angle*180/np.pi)
                rospy.loginfo("ogrid_resolution = %g", self.ogrid_resolution)
                rospy.loginfo("vector = [%g, %g]", vector[0], vector[1])


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


    def do_pca(self, cluster):
        # perform PCA analysis
        # (1) compute mean for each feature in the sample set
        dock_mean = np.mean(dock_cluster, axis=0)
        # mean normalization: replace each feature by feature - mean
        dock_cluster_norm = dock_cluster - dock_mean
        # (2) data covariance (first we have to transpose the data
        sigma = np.cov(dock_cluster_norm, rowvar=False, bias=True)
        # (3) singular value decomposition
        u, s, vh = np.linalg.svd(sigma)
        eigvector = u[:,0]*side
        angle = np.arctan2(eigvector[1], eigvector[0])
        dock_mean = dock_mean.astype(int)
        return eigvector, angle, dock_mean


if __name__ == "__main__":
    try:
        rospy.init_node("get_costmap_node", anonymous=True)
        getMap = GetMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        rospy.loginfo("ogrid to cv_image task has been terminated")







