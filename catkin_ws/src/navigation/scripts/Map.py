#!/usr/bin/env python

# Definitions for the search functionality

# @author: Patrick Gavigan

# Uses https://github.com/jrialland/python-astar
# Install using: pip install astar
# https://pypi.org/project/astar/

from astar import AStar
#import math
#import numpy as np
import nvector as nv
import re

class Map(AStar):
    
    # Define the map
    def __init__(self):
        
        #self.nodeNames = list()
        self.nodeLocations = dict()
        self.nodeGraph = dict()
        
        # AirSim map
        self.latLon = True
        f = open("D:/Local Documents/ROS_Workspaces/AirSimNavigatingCar/asl/map.asl", "r")
        
        # Grid world map
        #self.latLon = False
        #f = open("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/map.asl", "r")
            
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
        wgs84 = nv.FrameE(name='WGS84')
        
        n1Tuple = tuple(self.nodeLocations[n1])
        n1Coord = wgs84.GeoPoint(latitude=n1Tuple[0], longitude=n1Tuple[1], degrees = True)
        
        n2Tuple = tuple(self.nodeLocations[n2])
        n2Coord = wgs84.GeoPoint(latitude=n2Tuple[0], longitude=n2Tuple[1], degrees = True)
        
        delta = n1Coord.delta_to(n2Coord)
        distance = delta.length[0]
        return distance

    # Return the distance between two neighbouring nodes  
    # Assumes straight paths between locations (not necessarily true)
    def distance_between(self, n1, n2):
        return self.heuristic_cost_estimate(n1, n2)

    # Return list of neighbours
    def neighbors(self, node):
        return self.nodeGraph[node]


def test():
    map = Map()
    
    solutionPath = list(map.astar("post1","post3"))
    print(solutionPath)
    

if __name__ == '__main__':
    test()
        