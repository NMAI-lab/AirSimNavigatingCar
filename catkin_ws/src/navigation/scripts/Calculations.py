# -*- coding: utf-8 -*-
#"""
#Created on Fri Jul 10 20:26:41 2020
#
#@author: Patrick
#"""

from numpy import arctan2,sin,cos,degrees#, arctan, pi
import geopy.distance
#from geomag import geomag


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
#def correctDeclination(bearing, location):
#    gm = geomag.GeoMag()
#    mag = gm.GeoMag(location[0], location[1], 0)    # Assume altitude is 0
#    declination = mag.dec
#    correctBearing = (declination + bearing) % 360
#    return correctBearing

#def getMagneticBearing(a, b):
#    return correctDeclination(getBearing(a,b), b)


# https://cdn-shop.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
# Direction (y>0) = 90 - [arcTAN(x/y)]*180/pi
# Direction (y<0) = 270 - [arcTAN(x/y)]*180/pi
# Direction (y=0, x<0) = 180.0
# Direction (y=0, x>0) = 0.0  
#def getCompassAngle(x, y):
#    if y > 0:
#        return  90 - (arctan(x/y)) * 180 / pi   # Direction (y>0) = 90 - [arcTAN(x/y)]*180/pi
#    elif y < 0:
#        return 270 - (arctan(x/y)) * 180 / pi   # Direction (y<0) = 270 - [arcTAN(x/y)]*180/pi
#    elif ((y == 0) and (x < 0)):    # Direction (y=0, x<0) = 180.0
#        return 180.0
#    else:
#        return 0.0  # Direction (y=0, x>0) = 0.0

    
#def getRelativeBearing(current, destination):
#    relativeBearing = (destination - current) % 360
#    
#    while (relativeBearing > 180):
#        relativeBearing = relativeBearing - 360
#    
#    return relativeBearing

    
def tester():
    a = (47.641482370883864, -122.14036499180827)
    b = (47.64254900577103, -122.14036358759651)
    
    bearing = getBearing(a, b)
    print("bearing: " + str(bearing))

    #print("declination corrected bearing: " + str(correctDeclination(bearing, b)))
    
    #mag = 7.388
    #print("Relative bearing: " + str(getRelativeBearing(mag,bearing)))
    
    print("Range: " + str(getDistance(a,b)))
    
    
if __name__ == '__main__':
    tester()