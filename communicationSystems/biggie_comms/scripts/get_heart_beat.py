#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import requests
import datetime
import json
import time

#ipAdd = "192.168.65.2"
#portNum = "8080"
#courseLetter = "courseA"

ipAdd = "ec2-54-89-60-172.compute-1.amazonaws.com"
portNum = "8080"
courseLetter = "testCourse2"

lat = 0.0
lon = 0.0



def shutdownHook():
    print(stopRun(ipAdd, portNum, courseLetter))

#communication for starting a run
def startRun(ip, port, course, protocol = "http", teamCode = "FAU"):
    #setting up url path
    url = "{protocol}://{ip}:{port}/run/start/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, 
                                                                          course = course, teamCode = teamCode)
    
    
    #call POST command
    resp = requests.post(url = url,timeout=5)
    
    print(resp.text)
    #get response in JSON format
    #data = resp.json()
    
    return resp.text


#communication for stopping a run
def stopRun(ip, port, course, protocol = "http", teamCode = "FAU"):
    #setting up url path
    url = "{protocol}://{ip}:{port}/run/end/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, 
                                                                        course = course, teamCode = teamCode)
    print(url)
    #call POST command
    resp = requests.post(url = url, timeout=5)
    
    #get response in JSON format
    #data = resp.json()
    
    return resp.text


#commnunication for the heartbeat
def heartBeat(ip, port, course, challenge, lat, lon, protocol = "http", teamCode = "FAU"):
    #setting up url path
    url = "{protocol}://{ip}:{port}/heartbeat/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, 
                                                                          course = course, teamCode = teamCode)
    #set the content-type as 'application/json'
    headers = {'content-type': 'application/json'}
    
    #get the current timestamp without any spacing
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    
    #set up a dictionary for payload
    lib = {"timestamp": timestamp, 
            "challenge": challenge,
            "position":{"datum": "WGS84", "latitude": lat, "longitude": lon}}
    
    #convert payload with JSON
    PARAMS = json.dumps(lib)
    
    #call POST command to given url and send defined and headers/payloads
    resp = requests.post(url = url, headers = headers, data = PARAMS)
    
    #get response in JSON format
    data = resp.json()
    
    return data


#communication for follow the leader
def followLeader(ip, port, course, protocol = "http", teamCode = "FAU"):
    #setting up the url path
    url = "{protocol}://{ip}:{port}/followLeader/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, 
                                                                             course = course, teamCode = teamCode)

    #call the GET command
    resp = requests.get(url = url) 
    
    #get response in JSON format
    data = resp.json()
    
    return data


#communication for the docking
def docking(ip, port, course, protocol = "http", teamCode = "FAU"):
    #setting up url path
    url = "{protocol}://{ip}:{port}/docking/image/{course}/{teamCode}".format(protocol = protocol, ip = ip, port = port, course = course, teamCode = teamCode)
    
    print(url)
    
    #define file
    fileobj = open('7segment.jpg', 'rb')
    files = {'file' : ('7segment.jpg', fileobj)}
    
    #header
    header = {'Content-type': 'multipart/form-data', 'Expect': '100-continue'}
    
    #call POST command to send the image
    resp = requests.post(url = url, headers = header, files = files)

    #get the response in JSON format
    #data = resp.json()
        
    return resp.status_code


def stringCallback(data):
    response = data.data
    if(response == "speed_gate_mission"):
        heartBeat(ipAdd, portNum, courseLetter, "speed", lat, lon, protocol = "http", teamCode = "FAU")
        print("*** speed gate mission ***")
    elif(response == "follow_the_leader"):
        print("*** follow the leader mission ***")
        heartBeat(ipAdd, portNum, courseLetter, "follow", lat, lon, protocol = "http", teamCode = "FAU")
        time.sleep(3)
        followLeader(ipAdd, portNum, courseLetter, protocol="http", teamCode="FAU")
    elif(response == "acoustic_docking"):
        print("*** acoustic docking mission ***")
        heartBeat(ipAdd, portNum, courseLetter, "docking", lat, lon, protocol = "http", teamCode = "FAU")
        time.sleep(3)
        docking(ipAdd, portNum, courseLetter, protocol="http", teamCode="FAU")
    elif(response == "buoy_field"):
        print("*** buoy field mission ***")
        heartBeat(ipAdd, portNum, courseLetter, "path", lat, lon, protocol = "http", teamCode = "FAU")
    elif(response == "navigation_channel"):
        heartBeat(ipAdd, portNum, courseLetter, "auto", lat, lon, protocol = "http", teamCode = "FAU")
        print("*** navigation channel mission ***")
    else:
        heartBeat(ipAdd, portNum, courseLetter, "return", lat, lon, protocol = "http", teamCode = "FAU")
        print("*** return home ***")


                   
def stateCallback(data):
    lat = data.latitude
    lon = data.longitude


def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("hlp_out", String, stringCallback)
    rospy.Subscriber("fix", NavSatFix, stateCallback) 
    rospy.on_shutdown(shutdownHook)
    rospy.spin()


if __name__ == "__main__":
    #stopRun(ipAdd, portNum, courseLetter, teamCode = "GIT")
    print(startRun(ipAdd, portNum, courseLetter))
    listener()

