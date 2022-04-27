#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime
from CarStateMachine import CarStateMachine

mid = 0
mission = -1
stateMachine = CarStateMachine()

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
    global stopRange, speedSetting, stateMachine
    
    perception = (gps, lane, speed, compass, obstacle)
    action = stateMachine.update(perception)


    if (action != '') and (action != 'none'):
        act(action,actionsPublisher)
        sendMessage(action,outboxPublisher)
        reasoningStop = datetime.now()
    sendReasoningPerforamnce(reasoningStart, reasoningStop, reasoningRatePublisher)
    
   
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


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
