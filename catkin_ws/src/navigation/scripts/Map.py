#!/usr/bin/env python

# Definitions for the search functionality

# @author: Patrick Gavigan

# Uses https://github.com/jrialland/python-astar
# Install using: pip install astar
# https://pypi.org/project/astar/

from astar import AStar
import numpy as np
import nvector as nv
import re
import rospy
from std_msgs.msg import String

worldMap = ""

class Map(AStar):
    
    # Define the map
    def __init__(self, filePath, latLon = False):
        
        self.nodeLocations = dict()
        self.nodeGraph = dict()
        self.latLon = latLon
            
        f = open(filePath, "r")
        
        for line in f:
            if not ('*' in line or '//' in line):
                if "locationName" in line:
                    self.parseLocationName(line)
                elif "possible" in line:
                    self.parseNodeGraph(line)


    def parseLocationName(self, line):
        parameter = re.search('\((.*)\)', line).group(1)
        parameterList = parameter.split(",")
        locationName = parameterList[0]
        locationList = list()
                
        for item in parameterList:
            if item != locationName:
                cleaned = item.replace(" ", "")
                cleaned = cleaned.replace("[", "")
                cleaned = cleaned.replace("]", "")
                locationList.append(float(cleaned))
        
        self.nodeLocations[locationName] = locationList
        
    def parseNodeGraph(self, line):
        parameter = re.search('\((.*)\)', line).group(1)
        parameterList = parameter.split(",")
        a = parameterList[0]
        b = parameterList[1]
        
        if not a in self.nodeGraph.keys():
            self.nodeGraph[a] = list()
        
        if not b in self.nodeGraph[a]:
            self.nodeGraph[a].append(b)
            
            
    # Compute the distance between two location codes
    def heuristic_cost_estimate(self, n1, n2):
        
        if self.latLon:
            wgs84 = nv.FrameE(name='WGS84')
            n1Tuple = tuple(self.nodeLocations[n1])
            n1Coord = wgs84.GeoPoint(latitude=n1Tuple[0], longitude=n1Tuple[1], degrees = True)
            n2Tuple = tuple(self.nodeLocations[n2])
            n2Coord = wgs84.GeoPoint(latitude=n2Tuple[0], longitude=n2Tuple[1], degrees = True)
            delta = n1Coord.delta_to(n2Coord)
            distance = delta.length[0]
        else:
            n1Coord = np.array(tuple(self.nodeLocations[n1]))
            n2Coord = np.array(tuple(self.nodeLocations[n2]))
            distance = np.linalg.norm(n2Coord-n1Coord)
        
        return distance

    # Return the distance between two neighbouring nodes  
    # Assumes straight paths between locations (not necessarily true)
    def distance_between(self, n1, n2):
        return self.heuristic_cost_estimate(n1, n2)

    # Return list of neighbours
    def neighbors(self, node):
        return self.nodeGraph[node]


    # Set destination action
    def setDestination(self, data):
        self.destination = data.data
        rospy.loginfo("Destination set: " + self.destination)
        
    # Set destination action
    def getDirections(self, data, args):
        (publisher) = args
        start = data.data
        solutionPath = list(self.astar(start,self.destination))
        rospy.loginfo("Destination set: " + self.destination)
        publisher.publish(String(str(solutionPath)))


def rosMain():
    # Setup the map
    latLon = True
    filePath = "D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/map.asl"
    global worldMap
    worldMap =  Map(filePath, latLon)
    
    # Init the node
    rospy.init_node('map', anonymous=True)

    # Setup the publisher for the result
    publisher = rospy.Publisher('nagivation/path', String, queue_size=10)

    # Subscribe to actions, watch for setDest messages
    rospy.Subscriber('navigation/position', String, worldMap.getDirections, (publisher))    

    # Subscribe to actions, watch for setDest messages
    rospy.Subscriber('navigation/destination', String, worldMap.setDestination)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

def test():
    # AirSim map
    # latLon = True
    # filePath = "D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/map.asl"
        
    # Grid world map
    latLon = False
    filePath = "D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/map.asl"
    
    
    map = Map(filePath, latLon)
    
    #solutionPath = list(map.astar("post1","post3"))
    solutionPath = list(map.astar("a","d"))
    print(solutionPath)
    

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
        
        