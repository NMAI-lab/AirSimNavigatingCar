#!/usr/bin/env python

# Definitions for the search functionality

# @author: Patrick Gavigan

# Uses https://github.com/jrialland/python-astar
# Install using: pip install astar
# https://pypi.org/project/astar/

from astar import AStar
import math
import json
import numpy as np
import geopy.distance
import nvector as nv

class RouteSearcher(AStar):

    # Define the map
    def __init__(self):
        
        # Set a reference frame for nvector
        self.wgs84 = nv.FrameE(name='WGS84')
        
        # Load node graph
        f = open('nodeGraph.json')
        self.nodeGraph = json.load(f)
        
        # Load node locations
        f = open('nodeLocations.json')
        self.nodeLocations = json.load(f)
        
        # Load node names
        f = open('nodeNames.json')
        self.nodeNames = json.load(f)
                
        # assign a dummy value for the destination until we have one specified
        self.setDestination(-1)
        
        # Clean up the map format, tuples, distances.
        self.setupMap()
        
    # Clean up the map format, tuples, distances.
    def setupMap(self):
        
        # Make the location coords a dict of tuples using nvector objects
        for location in self.nodeNames:
            locationTuple = tuple(self.nodeLocations[location])
            locationCoord = self.wgs84.GeoPoint(latitude=locationTuple[0], longitude=locationTuple[1], degrees = True)
            self.nodeLocations[location] = locationCoord
            
        # Update distances between neighbouring nodes to be real (in meters)
        for location in self.nodeNames:
            locationCoord = self.nodeLocations[location]
            i = 0
            for destination in self.nodeGraph[location]:
                destinationName = destination[0]
                destinationCoord = self.nodeLocations[destinationName]
                delta = locationCoord.delta_to(destinationCoord)
                distance = delta.length[0]
                destination = (destinationName, distance)
                self.nodeGraph[location][i] = destination
                i += 1
        

     # Compute the distance between two location codes
    def heuristic_cost_estimate(self, n1, n2):
        locationCoord = self.nodeLocations[n1]
        destinationCoord = self.nodeLocations[n2]
        distance = geopy.distance.distance(locationCoord, destinationCoord)
        return distance

    # Return the distance between two neighbouring nodes  
    def distance_between(self, n1, n2):
        return [item for item in self.nodeGraph[n1] if item[0] == n2][0][1]

    # Return list of neighbours
    def neighbors(self, node):
        neighbourNodes = self.nodeGraph[node]
        neighbourNames = [a_tuple[0] for a_tuple in neighbourNodes]
        return neighbourNames
    
    # Returns the unit vector of the vector.
    def unitVector(self, vector):
        return vector / np.linalg.norm(vector)
    
    # Returns the angle in radians between vectors 'v1' and 'v2'::
    def anglBetweenRad(self, v1, v2):
        v1_u = self.unitVector(v1)
        v2_u = self.unitVector(v2)
        
        dot = np.dot(v1_u,v2_u)
        det = np.linalg.det([v1_u, v2_u])
        
        return math.atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
       
        
    def angleBetweenDeg(self, v1, v2):
        return self.anglBetweenRad(v1, v2) * 180 / np.pi
    
    # Returns a direction for where to go to continue on the journey
    def getNextPointBearing(self, previous, current, nextPoint):
        currentCoord = self.nodeLocations[current]
        previousCoord = self.nodeLocations[previous]
        nextPointCoord = self.nodeLocations[nextPoint]
        
        #current = np.array([xCurrent, yCurrent])
        #previous = np.array([xPrevious, yPrevious])
        #destination = np.array([xNextPoint, yNextPoint])
        
        #currentDirection = current - previous
        #desiredDirection = destination - current
 
        #print("current direction: " + str(currentDirection))
        #print("Desired direction: " + str(desiredDirection))
                
        #turnAngle = self.angleBetweenDeg(desiredDirection, currentDirection)
        #print("Turn angle: " + str(turnAngle))
        turnAngle = 1
        return turnAngle
    
    # get the bearing angle for the next step in the plan
    def getNextTurnAngle(self, solutionPath, previous):
        
        # Case where we are at the destination, nothing really needs to be done
        if len(solutionPath) == 1:
            return 0
        
        # len is greater than 1. We only really case about where to go next
        else:
            nextPoint = solutionPath[1]
            current = solutionPath[0]
            return self.getNextPointBearing(previous, current, nextPoint)        
    
    
    def getNextDirection(self, previous, current):
        # Deal with the special case where there is no destination set
        if self.destination == -1:
            return "direction(unknown,unknown)"
        
        # Deal with special case where we are already at the destination
        if current == self.destination:
            return "direction(" + str(self.destination) + ",arrived)"
    
        # Deal with special case where there is no previous location
        if (previous == -1) or ("unknown" in previous):
            # Try straight ahead, no other way to know what direction you are facing
            # TODO: Investigate if there are alternatives to this (perhaps pick something from the graph at random)
            return "direction(" + str(self.destination) + ",forward)"  
        
        solutionPath = list(self.astar(current,self.destination))
        bearing = self.getNextTurnAngle(solutionPath, previous)

        # print("bearing: " + str(bearing))
        
        # Deal with only positive numbers        
        while bearing < 0:
            bearing = bearing + 360
            
        # Deal with case where the bearing is too big
        while bearing > 360:
            bearing = bearing - 360
            
        print("bearing: " + str(bearing))

        # Case where it is more or less straight ahead
        if ((bearing >= (360 - 45)) or (bearing < 45)):
            return "direction(" + str(self.destination) + ",forward)"
        
        # Case where it is more or less the the right
        elif (bearing >= (90-45)) and (bearing < (90+45)):
            return "direction(" + str(self.destination) + ",right)"
        
        # Case where it is more or less behind
        elif (bearing >= (180-45)) and (bearing < (180+45)):
            return "direction(" + str(self.destination) + ",behind)"
        
        # Case where it is more or less to the left (all that is left)
        else:
            return "direction(" + str(self.destination) + ",left)"
        
    def setDestination(self, destination):
        if destination == -1:
            self.destination = -1
            return
        
        try:
            self.nodeLocations[destination]     # Check if the location exists
            self.destination = destination      # No exception, set the destination
        except:
            # print("No location " + str(destination))
            return
        