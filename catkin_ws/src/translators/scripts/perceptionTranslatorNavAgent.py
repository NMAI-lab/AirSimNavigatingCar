#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from navigation.msg import GPS
from std_msgs.msg import Float64
from threading import Semaphore    

noiseParams = 10

positionPerception = ""
compassPerception = ""
speedPerception = ""
lkaPerception = ""
obstaclePerception = ""
updateReady = [False,False,False,False,False]
positionIndex = 0
compassIndex = 1
speedIndex = 2
lkaIndex = 3
obstacleIndex = 4
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
    
def translateLka(data, args):
    (perceptionPublisher) = args
    global lkaPerception, updateReady, lkaIndex, sem
    lkaData = data.data   
    sem.acquire()
    lkaPerception = lkaData
    updateReady[lkaIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)
    
def translateObstacle(data, args):
    (perceptionPublisher) = args
    global obstaclePerception, updateReady, obstacleIndex, sem
    obstacle = data.data   
    sem.acquire()
    obstaclePerception = "obstacle({})".format(obstacle)
    updateReady[obstacleIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)    

# This is an asynch perception, send the update directly    
def translatePath(data, args):
    (perceptionsPublisher) = args
    path = data.data
    perceptionString = "path(" + path + ")"
    perceptionString = perceptionString.replace(" ","")
    perceptionString = perceptionString.replace("'","")
    perceptionString = perceptionString
    
    rospy.loginfo("Perceptions: " + str(perceptionString))
    perceptionsPublisher.publish(perceptionString) 
    
def generateNoise(numNoiseParams):
    noisePercept = ""
    for i in range(numNoiseParams):
        noisePercept = noisePercept + " noise" + str(i) + "(" + str(i) + ")"
    return noisePercept
    

def sendUpdate(publisher):
    global positionPerception, compassPerception, speedPerception, lkaPerception, obstaclePerception, updateReady, sem
    sem.acquire()    
    if not False in updateReady:
        perception = positionPerception + " " + compassPerception + " " + speedPerception + " " + lkaPerception + " " + obstaclePerception      
        global noiseParams
        perception = perception + generateNoise(noiseParams)
        rospy.loginfo(perception)
        publisher.publish(perception)
        updateReady = [False,False,False,False,False]
    sem.release()

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('positionTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('sensor/gps', GPS, translateGPS, (perceptionPublisher))
    rospy.Subscriber('sensor/compass', Float64, translateCompass, (perceptionPublisher))
    rospy.Subscriber('sensor/speed', Float64, translateSpeed, (perceptionPublisher))
    rospy.Subscriber('nagivation/path', String, translatePath, (perceptionPublisher))
    rospy.Subscriber('lka', String, translateLka, (perceptionPublisher))
    rospy.Subscriber('sensor/obstacle', Float64, translateObstacle, (perceptionPublisher))
    
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
