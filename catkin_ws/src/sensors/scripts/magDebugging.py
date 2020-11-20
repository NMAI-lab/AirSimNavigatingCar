# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 10:13:35 2020

@author: Patrick
"""

import nvector as nv
import math
import airsim
wgs84 = nv.FrameE(name='WGS84')

# Two locations on the map. One directly ahead of the other, where the car starts up.
post1 = wgs84.GeoPoint(latitude=47.641482370883864, longitude=-122.14036499180827, degrees = True)
post2 = wgs84.GeoPoint(latitude=47.6426242556, longitude=-122.140354517, degrees = True)
currentPosition = post1
targetPosition = post2
delta = currentPosition.delta_to(targetPosition)
targetBearing = delta.azimuth_deg[0]

client = airsim.CarClient()
client.confirmConnection()
magData = client.getMagnetometerData()
compassAngle = math.degrees(math.atan(0 - (magData.magnetic_field_body.y_val/magData.magnetic_field_body.x_val)))

data = client.getGpsData()
latitude = data.gnss.geo_point.latitude
longitude = data.gnss.geo_point.longitude


print(compassAngle)
print(targetBearing)
print(latitude)
print(longitude)


courseCorrection = targetBearing - compassAngle

turnSetting = courseCorrection/180

print(turnSetting)
