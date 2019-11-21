#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from scipy.spatial.distance import pdist
from geometry_msgs.msg import Pose2D
import math

from usv_ahc_py.msg import depth_points as depth_points_msg
from usv_ahc_py.msg import cluster as cluster_msg
from usv_ahc_py.msg import cluster_list as cluster_list_msg
from usv_ahc_py.msg import gate as gate_msg
from usv_ahc_py.msg import sorted_gates as sorted_gates_msg

import numpy as np

#this package is going to subscribe to the labeled persistance table and output a sorted list of gates, where the closest gate is the first set in the published topic  
#it will start be finding the white can buoy, once this is done, it will search the labeled persistance table for the closest red buoy
#once the entry gate is found, it will look for the next red buoy and corresponding closest green bouy
#this will continue until the closest corresponding buoy is blue, signifying the end gate
#special note will have to be paid to making this dynamic, where new gates are able to be added and sorted accordingly
class persistanceSortedGates():
    def __init__(self):
        rospy.init_node('persistence_table', anonymous=False)
        
        self.pubSortedGateList = rospy.Publisher('sortedGateList', sorted_gates_msg, queue_size=1)
        
        rospy.Subscriber('/persistanceLabeledClusterList', cluster_list_msg, self.labeled_cluster_list_callback)
        rospy.Subscriber('/vehicle_pose', Pose2D, self.pose_callback)
        rospy.Subscriber('/p3d_wamv_ned', Odometry, self.state_callback)
       
        rospy.on_shutdown(self.shutdownHook)
        
        self.stateData=Odometry()
        self.whiteBuoyFound=False
        self.startGateFound=False
        self.clusterListMsgCopy=cluster_list_msg()
        self.whiteIndex=-1#initialize to an out of bounds value for safety later
        self.startRedIndex=-1#initialize to an out of bounds value for safety later
        self.nextClosestRedIndex=-1#initialize to an out of bounds value for safety later
        self.nextClosestGreenIndex=-1#initialize to an out of bounds value for safety later
        self.closestRed=1000#initialize to something absurdly high
        self.closestGreen=1000#initialize to something absurdly high
        self.blueBouyFound=False
        self.blueIndex=-1
        rospy.spin()
    
    def pose_callback(self,Pose2D):
        self.x=Pose2D.x
        self.y=Pose2D.y
        self.theta=Pose2D.theta
        self.newPose = True

    def state_callback(self,Odometry):
        self.stateData.twist=Odometry.twist

    def labeled_cluster_list_callback(self,clusterListMsg):
        self.clusterListMsgCopy=clusterListMsg
        self.sortedGateList=sorted_gates_msg()#we start with a fresh list everytime we get a new cluster, since the incoming message is handling persistance we do not need to also handle it here
        self.blueBouyFound=False
        #begin be going through the list and finding the white buoy, we will repeat this every time to ensure we have the most up to date information
        #print "the white buoy is at"
        #print self.whiteIndex
        self.find_white_buoy()

        #now find the complete start set
        self.find_start_gate()
            
        #now fill in as many gates as possible until end gate
        #we know that another gate is possible if the remaining items in clusterListMsgCopy is even
        self.currentGateNumber=1#this is one since we already found the start gate, which would be number 0
        while(len(self.clusterListMsgCopy.cluster_list) % 2 == 0 and len(self.clusterListMsgCopy.cluster_list) > 0 and not self.blueBouyFound):
            self.find_gates()
            #check to see if there are only two buoys left and one of them is the blue buoy
            if (len(self.clusterListMsgCopy.cluster_list)==2):
                for i in range(len(self.clusterListMsgCopy.cluster_list)):
                    if(self.clusterListMsgCopy.cluster_list[i].label.data=='blue_can'):
                        self.blueBouyFound=True
                        #print "blue buoy found, lets get this last gate"
        
        #print len(self.sortedGateList.sorted_gate_array)
        #print len(self.clusterListMsgCopy.cluster_list)
        
        if(self.blueBouyFound==True):
            #prepare the last gate
            something =1
            self.find_blue_buoy()
            self.find_last_gate()
       
        #print len(self.sortedGateList.sorted_gate_array)
        #print len(self.clusterListMsgCopy.cluster_list)

        #print self.sortedGateList
        self.pubSortedGateList.publish(self.sortedGateList)

    def find_white_buoy(self):
        for i in range(len(self.clusterListMsgCopy.cluster_list)):
            if(self.clusterListMsgCopy.cluster_list[i].label.data=='white_can'):
                #print "finding white buoy"
                self.startGate=gate_msg()
                self.whiteBuoyFound=True
                self.whiteIndex=i
                self.startGate.buoyLocations[0]=self.clusterListMsgCopy.cluster_list[i].centroid
                self.startGate.buoyColors[0].data='white'
                #this is known, set let's go ahead and set this here
                self.startGate.buoyColors[1].data='red'
                self.startGate.gateNumber.data=0
        del(self.clusterListMsgCopy.cluster_list[self.whiteIndex])

    def find_start_gate(self):
        for i in range(len(self.clusterListMsgCopy.cluster_list)):
            if(self.clusterListMsgCopy.cluster_list[i].label.data=='red_can'):
                #check the distance between this buoy and the white buoy
                newDist=np.sqrt(np.square(self.startGate.buoyLocations[0].x-self.clusterListMsgCopy.cluster_list[i].centroid.x)+np.square(self.startGate.buoyLocations[0].y-self.clusterListMsgCopy.cluster_list[i].centroid.y))
                currDist=np.sqrt(np.square(self.startGate.buoyLocations[0].x-self.startGate.buoyLocations[1].x)+np.square(self.startGate.buoyLocations[0].y-self.startGate.buoyLocations[1].y))
                #save the smaller value
                if(newDist<currDist):
                    #print "setting new red start gate"
                    self.startRedIndex=i
                    self.startGate.buoyLocations[1]=self.clusterListMsgCopy.cluster_list[i].centroid
        
        del(self.clusterListMsgCopy.cluster_list[self.startRedIndex])
        self.startGateFound=True
        self.sortedGateList.sorted_gate_array.append(self.startGate)
        #print(len(self.sortedGateList.sorted_gate_array))

    def find_gates(self):
        #start by finding the closest red buoy to the starting red buoy, then find the closest green
        #needs to be check the distance between this buoy and the previous gate red buoy
        self.newGate=gate_msg()
        self.newGate.gateNumber.data=self.currentGateNumber
        self.closestRed=1000
        self.closestGreen=1000
        for i in range(len(self.clusterListMsgCopy.cluster_list)):
            if(self.clusterListMsgCopy.cluster_list[i].label.data=='red_can'):
                #check the distance between this buoy and the starting red buoy
                closestRedContender=np.sqrt(np.square(self.sortedGateList.sorted_gate_array[self.currentGateNumber-1].buoyLocations[1].x-self.clusterListMsgCopy.cluster_list[i].centroid.x)+
                                    np.square(self.sortedGateList.sorted_gate_array[self.currentGateNumber-1].buoyLocations[1].y-self.clusterListMsgCopy.cluster_list[i].centroid.y))
                
                #if the currently calculated distance to red buoy is less than a previous one, that is the new closest red
                if(closestRedContender<self.closestRed):
                    #print "we have a red winner"
                    self.nextClosestRedIndex=i
                    self.closestRed=closestRedContender
                    self.newGate.buoyLocations[1]=self.clusterListMsgCopy.cluster_list[i].centroid
                    self.newGate.buoyColors[1].data='red'

        #remove the closest red buoy from the list
        del(self.clusterListMsgCopy.cluster_list[self.nextClosestRedIndex])

        #now we look for the closest green
        for i in range(len(self.clusterListMsgCopy.cluster_list)):
            if(self.clusterListMsgCopy.cluster_list[i].label.data=='green_can'):
                #check the distance between this buoy and the starting red buoy
                closestGreenContender=np.sqrt(np.square(self.sortedGateList.sorted_gate_array[self.currentGateNumber-1].buoyLocations[0].x-self.clusterListMsgCopy.cluster_list[i].centroid.x)+
                                      np.square(self.sortedGateList.sorted_gate_array[self.currentGateNumber-1].buoyLocations[0].y-self.clusterListMsgCopy.cluster_list[i].centroid.y))
                
                #if the currently calculated distance to red buoy is less than a previous one, that is the new closest red
                if(closestRedContender<self.closestGreen):
                    #print "we have a green winner"
                    self.nextClosestGreenIndex=i
                    self.closestGreen=closestGreenContender
                    self.newGate.buoyLocations[0]=self.clusterListMsgCopy.cluster_list[i].centroid
                    self.newGate.buoyColors[0].data='green'

        #remove the closest red buoy from the list
        del(self.clusterListMsgCopy.cluster_list[self.nextClosestGreenIndex])

        self.sortedGateList.sorted_gate_array.append(self.newGate)
        self.currentGateNumber=self.currentGateNumber+1

    def find_blue_buoy(self):
        for i in range(len(self.clusterListMsgCopy.cluster_list)):
            if(self.clusterListMsgCopy.cluster_list[i].label.data=='blue_can'):
                #print "finding blue buoy"
                self.lastGate=gate_msg()
                self.blueBuoyFound=True
                self.blueIndex=i
                self.lastGate.buoyLocations[0]=self.clusterListMsgCopy.cluster_list[i].centroid
                self.lastGate.buoyColors[0].data='blue'
                #this is known, set let's go ahead and set this here
                self.lastGate.buoyColors[1].data='red'
                self.lastGate.gateNumber.data=self.currentGateNumber
        del(self.clusterListMsgCopy.cluster_list[self.blueIndex])

    def find_last_gate(self):
        for i in range(len(self.clusterListMsgCopy.cluster_list)):
            if(self.clusterListMsgCopy.cluster_list[i].label.data=='red_can'):
                #check the distance between this buoy and the blue buoy
                newDist=np.sqrt(np.square(self.lastGate.buoyLocations[0].x-self.clusterListMsgCopy.cluster_list[i].centroid.x)+np.square(self.lastGate.buoyLocations[0].y-self.clusterListMsgCopy.cluster_list[i].centroid.y))
                currDist=np.sqrt(np.square(self.lastGate.buoyLocations[0].x-self.lastGate.buoyLocations[1].x)+np.square(self.lastGate.buoyLocations[0].y-self.lastGate.buoyLocations[1].y))
                #save the smaller value
                if(newDist<currDist):
                    #print "setting new red last gate"
                    self.lastRedIndex=i
                    self.lastGate.buoyLocations[1]=self.clusterListMsgCopy.cluster_list[i].centroid
    
        del(self.clusterListMsgCopy.cluster_list[self.lastRedIndex])
        self.lastGateFound=True
        self.sortedGateList.sorted_gate_array.append(self.lastGate)
        #print(len(self.sortedGateList.sorted_gate_array))

    def shutdownHook(self):
        print "Shutting down persistence table"
    
if __name__ == '__main__':
    try:
        theTable=persistanceSortedGates()
    except rospy.ROSInterruptException:
        pass
