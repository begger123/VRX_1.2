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
        self.dock_around_pub = rospy.Publisher("/dock_around", Float32MultiArray, latch=True, queue_size=10)
        # opencv-ros bridge
        self.bridge = CvBridge()
        # class variables
        self.side = True    # true: right, false: left
        self.got_plackard = False
        self.got_dock = False
        self.around_pts_pix = np.zeros((4,2), np.int)
        self.around_pts_ned = np.zeros((4,2), np.int)
        self.plackard_mean = np.zeros((2,1), np.int)


    def plackard_callback(self, msg):
        self.side = msg.data


    def get_costmap_callback(self, msg):
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
        cost, label, center = cv2.kmeans(data, 2, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        # Now separate the data, Note the flatten()
        A = data[label.ravel()==0].astype(int)
        B = data[label.ravel()==1].astype(int)

        labels_img = np.zeros((height, width, 3), np.uint8)
        labels_img[A[:,0], A[:,1]] = (255,0,0)      # Blue
        labels_img[B[:,0], B[:,1]] = (0,255,0)      # Green

        # compute dock-cluster
        cy_min = np.min(center[:,1])    # minimum y-coordinate (columns direction) among clusters locations
        self.dock_center = 0
        self.dock_size = 0
        self.dock_mean = np.zeros((1, 2))
        eigvector1 = np.zeros((2, 1))
        side = 1
        if (cy_min < 450):
            # which cluster it is?
            dock_center = np.argmin(center[:,1])
            # how many dock-pixels do we have?
            dock_cluster = data[label.ravel()==dock_center]
            dock_size = np.size(dock_cluster)
            #  rospy.loginfo("Cy_min = %g", np.min(center[:,1]))
            #  rospy.loginfo("dock_size = %g", dock_size)
            #  rospy.loginfo("dock_center is: %g", dock_center)
            if (dock_size > 3600):
                # perform PCA analysis
                eigvector1, eigvector2, dock_angle1, dock_angle2, dock_mean = self.do_pca(dock_cluster, 1)
                if (self.side == 0):
                    eigvector1 = -eigvector1
                    eigvector2 = -eigvector2

                # draw image and show eigenvector
                vector = (eigvector1*100).astype(int) + dock_mean
                points = np.zeros((4,2), np.int)
                # compute starting path-point
                points[0,:] = vector;
                # compute stop point
                points[1,:] = (eigvector1*60).astype(int) + dock_mean
                # compute prior sk-point prior to dock_point
                points[2,:] = (eigvector1*50).astype(int) + dock_mean
                # compute and draw docking_point
                points[3,:] = (eigvector1*20).astype(int) + dock_mean
                # Draw all the points and convert them to NED coordinates
                self.points_ned = np.zeros((points.shape[0], 2))
                self.points_ned = self.convert_to_ned(points.astype(int), height, width)

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

                # Compute points around the dock
                l = 40
                s = 30
                self.dock_mean = dock_mean
                dock_angle1 = dock_angle1 - np.pi/2;
                around_pts = np.array([ [-s, l], [-s, -l], [s, -l], [s, l] ])
                rot_mat = np.array([ [np.cos(dock_angle1), np.sin(dock_angle1)], [-np.sin(dock_angle1), np.cos(dock_angle1)] ])
                self.around_pts_pix = np.dot(around_pts, rot_mat).astype(int) + self.dock_mean.astype(int)
                # scaled eigenvectors for plotting
                self.vec_dock1 = (eigvector1*60).astype(int) + self.dock_mean
                self.vec_dock2 = (eigvector2*60).astype(int) + self.dock_mean
                # convert dock around_points to NED
                self.around_pts_ned = self.convert_to_ned(self.around_pts_pix.astype(int), height, width)
                # publish via ROS
                dock_around_ned = Float32MultiArray()
                j = 0
                for i in range(self.around_pts_ned.shape[0]):
                    dock_around_ned.data.append(np.int(self.around_pts_ned[i,0]))
                    dock_around_ned.data.append(np.int(self.around_pts_ned[i,1]))
                self.dock_around_pub.publish(dock_around_ned)

                # Drawing stuff onto opencv image
                # For drawing the line we have to invert the coordinates
                # Draw eigenvectors
                #  rospy.loginfo("dock_mean = [%g, %g]", self.dock_mean[0], self.dock_mean[1])
                #  rospy.loginfo("vec_dock = [%g, %g]", self.vec_dock1[0], self.vec_dock1[1])
                #  cv2.line(labels_img, (self.dock_mean[1], self.dock_mean[0]), (self.vec_dock1[1], self.vec_dock1[0]), (255,0,0), 2)
                #  cv2.line(labels_img, (self.dock_mean[1], self.dock_mean[0]), (self.vec_dock2[1], self.vec_dock2[0]), (0,255,0), 2)
                # Draw the dock_path
                #  cv2.line(labels_img, (dock_mean[1], dock_mean[0]), (vector[1], vector[0]), (255,255,255), 2)
                #  # Draw points on the dock_path
                #  for i in range(points.shape[0]):
                #      cv2.circle(labels_img, (points[i,1], points[i,0]), 4, (255,0,255), -1)

                # Draw points on the dock_path
                #  cv2.circle(labels_img, (points[0,1], points[0,0]), 4, (255,0,0), -1)
                #  cv2.circle(labels_img, (points[1,1], points[1,0]), 4, (0,255,0), -1)
                #  cv2.circle(labels_img, (points[2,1], points[2,0]), 4, (0,0,255), -1)
                #  cv2.circle(labels_img, (points[3,1], points[3,0]), 4, (255,0,255), -1)

                #  # Draw dock around_points
                #  cv2.circle(labels_img, (self.around_pts_pix[0,1], self.around_pts_pix[0,0]), 4, (255,0,0), -1)
                #  cv2.circle(labels_img, (self.around_pts_pix[1,1], self.around_pts_pix[1,0]), 4, (0,255,0), -1)
                #  cv2.circle(labels_img, (self.around_pts_pix[2,1], self.around_pts_pix[2,0]), 4, (0,0,255), -1)
                #  cv2.circle(labels_img, (self.around_pts_pix[3,1], self.around_pts_pix[3,0]), 4, (255,0,255), -1)

                # debug printouts
                #  rospy.loginfo("dock_centroid = [%g, %g]", dock_mean[0], dock_mean[1])
                #  rospy.loginfo("eigvector = [%g, %g]", eigvector[0], eigvector[1])
                #  rospy.loginfo("angle = %g deg.", angle*180/np.pi)
                #  rospy.loginfo("ogrid_resolution = %g", self.ogrid_resolution)
                #  rospy.loginfo("vector = [%g, %g]", vector[0], vector[1])

        #  # draw dock_paths
        #  cv2.imshow("Labels Image", labels_img)
        #  cv2.waitKey(30)


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








