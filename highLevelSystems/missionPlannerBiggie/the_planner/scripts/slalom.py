#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point32
from usv_ahc_py.msg import buoy as buoy_msg
from usv_ahc_py.msg import look_up_table as look_up_table_msg
from the_planner.msg import waypoint_list as waypoint_list_msg
import numpy as np

pubSlalom = rospy.Publisher('slalom_waypoints', waypoint_list_msg, queue_size=10)

def shutdownHook():
    print "Shutting down slalom generator"

def mirror(waypoint_list_msg_t):
	mirrored_point_t=Point32()
    #calculate the y-intercept between waypoint 3 and 4
	#the 0.001 term below is a small epsilon to account for the possibility of dividing by zero
	m34 = (waypoint_list_msg_t.waypoint_list[2].y - waypoint_list_msg_t.waypoint_list[3].y)/(waypoint_list_msg_t.waypoint_list[2].x - waypoint_list_msg_t.waypoint_list[3].x)
	b34 = waypoint_list_msg_t.waypoint_list[2].y - m34*waypoint_list_msg_t.waypoint_list[2].x

	#use the first original waypoints X coordinate, and then calculate the new
	#Y coordinate for the return path ending based on said X, the slope (m23)
	#and the y-intercept (b34)
	mirrored_point_t.x=waypoint_list_msg_t.waypoint_list[0].x
	mirrored_point_t.y=(m34)*waypoint_list_msg_t.waypoint_list[0].x + b34
	return mirrored_point_t

def reverse(wps):
    #this function takes the calculated forward trajectory (wps) and simply 
    #reverses the predetermined waypoints to yield the return trip

    #reverse the waypoints between 2 and end - 2
    goBack = wps[2:end-2]
    temp = fliplr(wps[2:end-2])

    #reverse pair orders between waypoints 2 and 9 to get return trajectory
    k = 1
    while k < len(temp):

        #reverse pairing
        goBack[k] = temp[k+1]
        goBack[k+1] = temp[k]

        #increment k to get next bouy pair
        k = k + 2

    end

    #tack on the last point of original waypoints to get full path
    goBack = sum(wps(end), goBack)

    #calculate the mirrored point of first original waypoint to get full path
    goBack = mirror(wps, goBack)

def waypoint(B1, B2, dist, bouysSeen, direction):
	wps1=Point32()
	wps2=Point32()
	wps3=Point32()
	wps4=Point32()
	waypoint_list_msg_t=waypoint_list_msg()
	##inputs: 

	#B1: the closest bouy to the vehicle

	#B2: the farther bouy from the vehicle

	#dist: the radial distance from the bouy that the way point will be
	#generated

	#bouysSeen: the count of bouys that have been seen so far

	#direction: variable used to make sure the waypoints are in the correct order for
	#the boat to follow (traverse left or right)

	## outputs: 

	#waypoint_list_msg_t: the waypoint(s) generated with the Bouy pair

	## generate the waypoints 

	#make dist positive or negative depending on direction of line travel
	if(B1.centroid.x > B2.centroid.x):
	    dist = -dist

	#get the line characteristics between the two bouys
	m = (B1.centroid.y - B2.centroid.y)/(B1.centroid.x - B2.centroid.x) #slope
	b = B1.centroid.y - m*B1.centroid.x #intercept

	#parallel line intercepts
	p1 = b - 2*dist
	p2 = b + 2*dist

	#If it is the first bouy, generate 3 waypoints
	if(bouysSeen == 0):
		#place the 3 waypoints
		if B1.color =='red':
		    #waypoint to the right of the bouy
			wps1.x=B1.centroid.x-dist
			wps1.y=m*(B1.centroid.x - dist) + p1
		else:
		    #waypoint to the left of the bouy
			wps1.x=B1.centroid.x-dist
			wps1.y=m*(B1.centroid.x - dist) + p2

		wps2.x=B1.centroid.x+dist
		wps2.y=m*(B1.centroid.x + dist) + p1	    
		wps3.x=B1.centroid.x+dist
		wps3.y=m*(B1.centroid.x + dist) + p2

		#waypoint vector for 3 element case
		waypoint_list_msg_t.waypoint_list.append(wps1)
		waypoint_list_msg_t.waypoint_list.append(wps2)
		waypoint_list_msg_t.waypoint_list.append(wps3)

	#If it is the 5th bouy, then place 4 total waypoints
	elif(bouysSeen == 4):
		#place the waypoints
		wps1.x=B1.centroid.x+dist
		wps1.y=m*(B1.centroid.x + dist) + p1
		wps2.x=B1.centroid.x+dist
		wps2.y=m*(B1.centroid.x + dist) + p2	    
		wps3.x=B2.centroid.x+dist
		wps3.y=m*(B2.centroid.x + dist) + p1	    
		wps4.x=B2.centroid.x+dist
		wps4.y=m*(B2.centroid.x + dist) + p2

		#waypoint vector for 4 element case, make sure the points are stored in
		#the correct order
		if(direction%2 != 0):
			waypoint_list_msg_t.waypoint_list.append(wps1)
			waypoint_list_msg_t.waypoint_list.append(wps2)
			waypoint_list_msg_t.waypoint_list.append(wps3)
			waypoint_list_msg_t.waypoint_list.append(wps4)
		else:
			waypoint_list_msg_t.waypoint_list.append(wps2)
			waypoint_list_msg_t.waypoint_list.append(wps1)
			waypoint_list_msg_t.waypoint_list.append(wps3)
			waypoint_list_msg_t.waypoint_list.append(wps4)

	#All other bouy pairs just need two waypoints calculated
	else:
		#place the 2 waypoints
		wps1.x=B1.centroid.x+dist
		wps1.y=m*(B1.centroid.x + dist) + p1
		wps2.x=B1.centroid.x+dist
		wps2.y=m*(B1.centroid.x + dist) + p2	

		#waypoint vector for 2 element case, make sure the points are stored in
		#the correct order
		if(direction%2 != 0):
			waypoint_list_msg_t.waypoint_list.append(wps1)
			waypoint_list_msg_t.waypoint_list.append(wps2)
		else:
			waypoint_list_msg_t.waypoint_list.append(wps2)
			waypoint_list_msg_t.waypoint_list.append(wps1)

	return waypoint_list_msg_t

def path(look_up_table_msg_t,dist,bouysSeen,direction):

	#UNTITLED3 Summary of this function goes here
	#   Detailed explanation goes here
	waypoint_list_msg_t=waypoint_list_msg()
	for i in range(len(look_up_table_msg_t.labeled_buoy_list)-1):
		waypoint_list_msg_temp_t=waypoint_list_msg()
		#call function to calculate the waypoints with the dist set to 2 meters in x and y
		waypoint_list_msg_temp_t=waypoint(look_up_table_msg_t.labeled_buoy_list[i], look_up_table_msg_t.labeled_buoy_list[i+1], dist , bouysSeen, direction)

		waypoint_list_msg_t.waypoint_list.extend(waypoint_list_msg_temp_t.waypoint_list)

		#increase the bouy count
		if(bouysSeen == 0):
			bouysSeen = bouysSeen + 2
		else:
			bouysSeen = bouysSeen + 1

		#update direction number
		direction = direction +1

		#store calculated waypoints into the main array
		#wps = sum(wps, calc)
		#publish returned waypoints

	return waypoint_list_msg_t

def look_up_table_callback(look_up_table_msg_t):
	#pass objects to path
	#print(look_up_table_msg_t)
	buoysSeen=0
	waypoint_list_msg_there_t=waypoint_list_msg()
	waypoint_list_msg_there_t=path(look_up_table_msg_t,2,buoysSeen,direction)
	
	waypoint_list_msg_back_t=waypoint_list_msg()
	reversed_look_up_table_msg_t=look_up_table_msg()
	reversed_look_up_table_msg_t.labeled_buoy_list=look_up_table_msg_t.labeled_buoy_list[::-1]

	buoysSeen=len(reversed_look_up_table_msg_t.labeled_buoy_list)
	waypoint_list_msg_back_t=path(reversed_look_up_table_msg_t,2,buoysSeen,direction)

	waypoint_list_msg_back_t.waypoint_list.extend([mirror(waypoint_list_msg_there_t)])

	waypoint_list_msg_there_t.waypoint_list.extend(waypoint_list_msg_back_t.waypoint_list)
	pubSlalom.publish(waypoint_list_msg_there_t)

def look_up_table_sub():
	rospy.init_node('slalom_generator', anonymous=False)
	rospy.Subscriber('/look_up_table', look_up_table_msg, look_up_table_callback)
	rospy.on_shutdown(shutdownHook)
	rospy.spin()

if __name__ == '__main__':
	try:
		direction=1
		look_up_table_sub()
	except rospy.ROSInterruptException:
		pass