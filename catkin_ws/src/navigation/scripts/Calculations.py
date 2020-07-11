# -*- coding: utf-8 -*-
"""
Created on Fri Jul 10 20:26:41 2020

@author: Patrick
"""

from numpy import arctan2,sin,cos,degrees
import geopy.distance
from geomag import geomag

def getDistance(a,b):
    return geopy.distance.distance(a, b)


# https://towardsdatascience.com/calculating-the-bearing-between-two-geospatial-coordinates-66203f57e4b4
def getBearing(a, b):
    
    aLat = a[0]
    aLon = a[1]
    bLat = b[0]
    bLon = b[1]
    
    dL = bLon - aLon
    X = cos(bLat) * sin(dL)
    Y = cos(aLat) * sin(bLat) - sin(aLat) * cos(bLat) * cos(dL)
    bearing = arctan2(X, Y)
    bearing = ((degrees(bearing) + 360) % 360)
    return bearing


# Correct the bearing for declenation error
def correctDeclination(bearing, location):
    gm = geomag.GeoMag()
    mag = gm.GeoMag(location[0], location[1], 0)    # Assume altitude is 0
    declination = mag.dec
    correctBearing = declination + bearing
    return correctBearing

def getTrueBearing(a, b):
    return correctDeclination(getBearing(a,b), b)


def tester():
    a = (47.641482370883864, -122.14036499180827)
    b = (47.64254900577103, -122.14036358759651)
    
    bearing = getBearing(a, b)
    print("bearing: " + str(bearing))

    print("declination corrected bearing: " + str(correctDeclination(bearing, b)))
    
    print("Range: " + str(getDistance(a,b)))
    
    
if __name__ == '__main__':
    tester()