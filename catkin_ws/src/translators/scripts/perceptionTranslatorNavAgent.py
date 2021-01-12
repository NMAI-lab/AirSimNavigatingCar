#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from navigation.msg import GPS
from std_msgs.msg import Float64
from threading import Semaphore    

positionPerception = ""
compassPerception = ""
speedPerception = ""
updateReady = [False,False,False]
positionIndex = 0
compassIndex = 1
speedIndex = 2
sem = Semaphore()

def translateGPS(data, args):
    (perceptionPublisher) = args
    global positionPerception, updateReady, positionIndex, sem
    position = (data.latitude, data.longitude)
    sem.acquire()
    positionPerception = "gps{}".format(position)
    updateReady[positionIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)

def translateCompass(data, args):
    (perceptionPublisher) = args
    global compassPerception, updateReady, compassIndex, sem
    compass = data.data
    sem.acquire()
    compassPerception = "compass({})".format(compass)
    updateReady[compassIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)
    
def translateSpeed(data, args):
    (perceptionPublisher) = args
    global speedPerception, updateReady, speedIndex, sem
    speed = data.data   
    sem.acquire()
    speedPerception = "speed({})".format(speed)
    updateReady[speedIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)

# This is an asynch perception, send the update directly    
def translatePath(data, args):
    (perceptionsPublisher) = args
    path = data.data
    perceptionString = "path(" + path + ")"
    perceptionString = perceptionString.replace(" ","")
    perceptionString = perceptionString.replace("'","")
    
    rospy.loginfo("Perceptions: " + str(perceptionString))
    perceptionsPublisher.publish(perceptionString) 

def sendUpdate(publisher):
    global positionPerception, compassPerception, speedPerception, updateReady, sem
    sem.acquire()    
    if not False in updateReady:
        perception = positionPerception + " " + compassPerception + " " + speedPerception        
        rospy.loginfo(perception)
        publisher.publish(perception)
        updateReady = [False, False, False]
    sem.release()

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('positionTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('sensor/gps', GPS, translateGPS, (perceptionPublisher))
    rospy.Subscriber('sensor/compass', Float64, translateCompass, (perceptionPublisher))
    rospy.Subscriber('sensor/speed', Float64, translateSpeed, (perceptionPublisher))
    rospy.Subscriber('nagivation/path', String, translatePath, (perceptionPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
