#! /usr/bin/env python
import sys
sys.path.insert(0,'/home/eric/ros/VRX_workspace/src/VRX_1.2/highLevelSystems/yolo_detection/')

from darkflow.net.build import TFNet
import cv2
import threading
import time
import rospy
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ObjectRecognizer():
	def __init__(self):
		self.detectionImage = self.image = None
		self.lastTime = time.time()
		self.elapsedTime = 1
		self.boxes = []

		script_path = os.path.dirname(os.path.realpath(__file__))
		print script_path

		self.options = {"model": os.path.join(script_path, "placard.cfg"), 
				"load": os.path.join(script_path, "placard_10000.weights"), 
				"gpu": 1.0,
				"threshold": 0.40, 
				"config":script_path}

		self.tfnet = TFNet(self.options)
		self.colors = self.tfnet.meta['colors']
		self.classesColorMap = {}
		self.newImageFlag = False

		# Start ROS
		rospy.init_node("object_recognizer")
		self.bridge = CvBridge()
		self.imagePub = rospy.Publisher("/yolo", Image, queue_size = 1)
		
		print("publisher created")

		self.imageSub = rospy.Subscriber("/wamv/sensors/cameras/front_left_camera/image_raw", Image, self.newColorImage)

		print("subscriber created")

		self.rate = rospy.Rate(10)

		#while self.image == None: 
		#	self.rate.sleep()
		#print('past none check')
		#self.liveThread = threading.Thread(target=self.liveRecognitionThread)

		rospy.spin()


	def newColorImage(self, imageMsg):
		self.image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(imageMsg),cv2.COLOR_RGB2BGR)
		#self.liveThread.start()
		self.liveRecognitionThread()
		self.mainThread()

	def getClassColor(self, className):
		if className in self.classesColorMap:
			return self.classesColorMap[className]
		self.classesColorMap[className] = self.colors[len(self.classesColorMap)]

	def mainThread(self):
		h, w, _ = self.image.shape
		#while not rospy.is_shutdown():
		self.detectionImage = self.image.copy()
		for bbox in self.boxes:
			left, top, right, bot, label = bbox['topleft']['x'], bbox['topleft']['y'], bbox['bottomright']['x'], bbox['bottomright']['y'], bbox['label']
			color = self.getClassColor(label)
			cv2.rectangle(self.detectionImage, (left, top),(right, bot), color, 3)
			cv2.putText(self.detectionImage, label, (left, top - 12),0, 2e-3 * h,
color, 1)
		self.imagePub.publish(self.bridge.cv2_to_imgmsg(self.detectionImage,
"bgr8"))
		self.rate.sleep()

	def liveRecognitionThread(self):

		#while not rospy.is_shutdown():
		self.lastTime = time.time()
		self.boxes = self.tfnet.return_predict(self.image)
		print "NEW FRAME"
		print(self.boxes)
		self.elapsedTime = time.time() - self.lastTime


if __name__ == "__main__":
	try:
		recognizer = ObjectRecognizer()
	except Exception as e:
		print e
		rospy.signal_shutdown('Testing')
