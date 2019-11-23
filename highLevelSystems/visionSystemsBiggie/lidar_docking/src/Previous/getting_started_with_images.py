#!/usr/bin/env python
import rospy
import cv2

print("Hello ve!")
img = cv2.imread("/home/armajoe/Dropbox/Documents/ROS/Miscellaneous/tutorials_ws/src/opencv/src/baboon.png", 0)
print(img)
print("OpenCV is working !!!")
cv2.imshow("mywindow", img)
cv2.waitKey(3000)
cv2.destroyAllWindows()

rospy.init_node('opencv_example', anonymous=True)
rospy.loginfo("Hello ROS ve!")
