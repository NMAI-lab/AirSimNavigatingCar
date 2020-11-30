#!/usr/bin/env python

# Definitions for the search functionality

# @author: Patrick Gavigan

# Uses https://github.com/jrialland/python-astar
# Install using: pip install astar
# https://pypi.org/project/astar/

from astar import AStar
#import math
import json
#import numpy as np
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
        delta = locationCoord.delta_to(destinationCoord)
        distance = delta.length[0]
        return distance

    # Return the distance between two neighbouring nodes  
    def distance_between(self, n1, n2):
        return [item for item in self.nodeGraph[n1] if item[0] == n2][0][1]

    # Return list of neighbours
    def neighbors(self, node):
        neighbourNodes = self.nodeGraph[node]
        neighbourNames = [a_tuple[0] for a_tuple in neighbourNodes]
        return neighbourNames
    
    # Returns a direction for where to go to continue on the journey
    def getNextPointBearing(self, bearing, currentCoord, nextPoint):
#        currentCoord = self.nodeLocations[current]
#        previousCoord = self.nodeLocations[previous]
        nextPointCoord = self.nodeLocations[nextPoint]
        
        #deltaCurrent = previousCoord.delta_to(currentCoord)
        azCurrent = bearing #deltaCurrent.azimuth_deg
        deltaNext = currentCoord.delta_to(nextPointCoord)
        azNext = deltaNext.azimuth_deg
        
        turnAngle = azNext - azCurrent
        return turnAngle
    
    # get the bearing angle for the next step in the plan
    def getNextTurnAngle(self, solutionPath, current, bearing):
        
        # Case where we are at the destination, nothing really needs to be done
        if len(solutionPath) == 1:
            return [0]
        
        # len is greater than 1. We only really case about where to go next
        else:
            # Add a check to make sure that the nextPoint is not where we are now?
            nextPoint = solutionPath[1]
            #current = solutionPath[0]
            return self.getNextPointBearing(bearing, current, nextPoint)        


    # Return the range (in m) to a location specified by locationCode and a 
    # coordinate    
    def rangeToLocation(self, locationCode, coordinate):
        locationCoordinate = self.nodeLocations[locationCode]
        delta = coordinate.delta_to(locationCoordinate)
        distance = delta.length[0]
        bearing = delta.azimuth_deg[0]
        return (distance, bearing)
    
    # Find the location nearest to the current position. Returns the name and 
    # range (m) to that location as a tuple: (name, range)
    def getNearestLocationAndRange(self, position):
        nearestRange = 9000000          # Arbitrary large number to start us with
        nearestName = "meow"            # Arbitrary starting name
        nearestBearing = 0
        
        for locationName in self.nodeNames:
            (thisRange, thisBearing) = self.rangeToLocation(locationName, position)
            
            if (thisRange < nearestRange) and abs(thisBearing) < 100 :
                nearestRange = thisRange
                nearestBearing = thisBearing
                nearestName = locationName
            
        return (nearestName, nearestRange, nearestBearing)
            
    
    def getNextDirection(self, current, bearing):
        # Deal with the special case where there is no destination set
        if self.destination == -1:
            return ("direction(unknown,unknown,0)", "unknown", 0)
        
        # Get the name and range to the nearest codded location that is generally in front of us
        (nearestLocationName, rangeToNearest, nearestBearing) = self.getNearestLocationAndRange(current)
        
        # Deal with special case where we are already at the destination
        # (If we are withing a meter of the destination, close enough)
        if ((self.destination in nearestLocationName) and (rangeToNearest < 30)):
            return ("direction(" + str(self.destination) + ",arrived,0)", nearestLocationName, rangeToNearest)
    
        # Deal with special case where there is no previous location or we 
        # are not near any specific codded location
        # Just drive forward to get to a codded location
        #if ((current.delta_to(previous) < 5) or (rangeToNearest > 30)):
        if (rangeToNearest > 40):
            return ("direction(" + str(self.destination) + ",forward, 0)", nearestLocationName, rangeToNearest)
        
        #print("HERE!!!!!!!!!")
        
        # We are near a codded location. Get directions
        solutionPath = list(self.astar(nearestLocationName,self.destination))
        bearing = self.getNextTurnAngle(solutionPath, current, bearing)[0]
    
        waypoint = ""
        if len(solutionPath) > 1 and rangeToNearest < 50 and (nearestLocationName in solutionPath[0]):
            waypoint = "waypoint(" + solutionPath[0] + "," + solutionPath[1] + ") "

        print("bearing: " + str(bearing))
        
        # Don't have a bearing smaller than -180        
        while bearing < -180:
            bearing = bearing + 360
            
        # Deal with case where the bearing is too big
        while bearing > 180:
            bearing = bearing - 360
            
        print("bearing: " + str(bearing))

        # Case where it is more or less straight ahead
        if abs(bearing) <= 45:
            directions = "direction(" + str(self.destination) + ",forward," + str(bearing) + ")"
        
        # Case where it is more or less the the right
        elif bearing > 45:
            directions = "direction(" + str(self.destination) + ",right," + str(bearing) + ")"
        
        # Case where it is more or less behind
        #elif (bearing >= (180-45)) and (bearing < (180+45)):
        #    directions = "direction(" + str(self.destination) + ",behind," + str(bearing) + ")"
        
        # Case where it is more or less to the left (all that is left)
        else:
            directions = "direction(" + str(self.destination) + ",left," + str(bearing) + ")"
        
        directions = waypoint + directions
        
        return (directions, nearestLocationName, rangeToNearest)
        
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
        