#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime
import nvector as nv

stopRange = 5.0
speedSetting = 0
mid = 0
mission = -1
wgs84 = nv.FrameE(name='WGS84')
destination = wgs84.GeoPoint(latitude=6426242556, longitude=-122.140354517, degrees = True)

def receiveMessage(data, args):
    (outboxPublisher,_,_) = args
    global mission
    if "mission" in data.data:
        sendMessage("mission(navigate,post2)",outboxPublisher)
        sendMessage("navigate(post2)", outboxPublisher)
        mission = "mission(navigate,post2)"
    
    
def perceive(data, args):
    perceptionString = data.data
    (outboxPublisher, actionsPublisher, reasoningRatePublisher) = args
    reasoningStart = datetime.now()
    (gps,compass,lane,speed,obstacle) = extractPerceptions(perceptionString)
    
    global mission
    if mission != -1:
        decide(gps,compass,lane,speed,obstacle,reasoningStart,outboxPublisher, actionsPublisher, reasoningRatePublisher)

def extractPerceptions(perceptionString):
    (gps,compass,lane,speed,obstacle) = (0,0,0,0,0)
    perceptList = perceptionString.split(')')
    
    for p in perceptList:
        if '(' in p:
            values = p.split('(')[1]
            if 'gps' in p:
                lat = float(values.split(', ')[0])
                lon = float(values.split(', ')[1])
                gps = (lat,lon)
            elif 'compass' in p:
                compass = float(values)
            elif 'lane' in p:
                lkaParams = values.split(',')
                lane = tuple([float(i) for i in lkaParams])
            elif 'speed' in p:
                speed = float(values)
            elif 'obstacle' in p:
                obstacle = float(values)
            
    return (gps,compass,lane,speed,obstacle)


def decide(gps,compass,lane,speed,obstacle,reasoningStart,outboxPublisher,actionsPublisher,reasoningRatePublisher):
    global stopRange, speedSetting
    
    action = ''
    if (obstacle < stopRange):
        action = 'setSpeed(0)'
    elif speedSetting == 0:
        speedSetting = 8
        action = 'setSpeed(' + str(speedSetting) + ')'
    else:
        (lkaSteering,_,_,c,d) = lane
        if ((c != 0) or (d != 0)):
            action = 'steering(' + str(lkaSteering) + ')'
        else:
            compassSteering = getCompassSteering(gps,compass)
            action = 'steering(' + str(compassSteering) + ')'


    if action != '':
        act(action,actionsPublisher)
        sendMessage(action,outboxPublisher)
        reasoningStop = datetime.now()
    sendReasoningPerforamnce(reasoningStart, reasoningStop, reasoningRatePublisher)


def getCompassSteering(gps,compass):
    global destination, wgs84
    declanation = 7.5

    (curLat,curLon) = gps
    current = wgs84.GeoPoint(latitude=curLat, longitude=curLon, degrees = True)
    destinationBearing = destination.delta_to(current).azimuth_deg[0]
    courseCorrection = destinationBearing - (compass + declanation)

    if courseCorrection >= 20:
        steeringSetting = 1
    elif courseCorrection <= -20:
        steeringSetting = -1
    else:
        steeringSetting = courseCorrection/180
    
    return steeringSetting
    
   
def act(action, publisher):
    rospy.loginfo("Action: " + str(action))
    publisher.publish(action)
    
    
def sendMessage(message, publisher):
    global mid
    output = "<mid" + str(mid) + ",0,tell,BROADCAST," + message + ">"
    rospy.loginfo("Outbox: " + str(output))
    publisher.publish(output)
    mid = mid + 1
    
def sendReasoningPerforamnce(start, stop, publisher):
    reasoningTime = stop - start
    publisher.publish(str(reasoningTime))


# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('traditionalAgent', anonymous=True)
    
    outboxPublisher = rospy.Publisher('outbox', String, queue_size=10)
    actionsPublisher = rospy.Publisher('actions', String, queue_size=10)
    reasoningRatePublisher = rospy.Publisher('reasoningPerformance', String, queue_size=10)

    publishers = (outboxPublisher, actionsPublisher, reasoningRatePublisher)

    rospy.Subscriber('inbox', String, receiveMessage, publishers)
    rospy.Subscriber('perceptions', String, perceive, publishers)
    
    rospy.spin()

#def test():
#    perceptionString = "gps(47.64148237132387, -122.1403649909945) compass(-8.76061640298) speed(0.0) lane(0.0,0.0,0.0,0.0,0.0) obstacle(32.6919536591)"
#    (gps,compass,lane,speed,obstacle) = extractPerceptions(perceptionString)
#    steering = getCompassSteering(gps,compass)

if __name__ == '__main__':
#    test()
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
