#!/usr/bin/env python
from std_msgs.msg import Float64

import rospy
import airsim
import numpy

def parse_lidarData(data):
    points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0]/3), 3))
    return points

def obstacleSensor():
    pub = rospy.Publisher('sensor/obstacle', Float64, queue_size=1)
    rospy.init_node('obstacle', anonymous=True)
    rate = rospy.Rate(60)

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()
        
    while not rospy.is_shutdown():

        # Get the LIDAR data
        lidarData = client.getLidarData();
        distance = 0
        if (len(lidarData.point_cloud) < 3):
            distance = -1
        else:
            points = parse_lidarData(lidarData)
            x =[]
            for i in points[:4]:
                x.append(i[0])
                distance = sum(x) / len (x)

        print(x)
        rospy.loginfo(distance)
        pub.publish(distance)       # Obstacle close if distance < 10

        rate.sleep()
                    

if __name__ == '__main__':
    try:
        obstacleSensor()
    except rospy.ROSInterruptException:
        pass
