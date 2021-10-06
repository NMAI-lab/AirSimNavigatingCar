#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime


mid = 0
mission = -1

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
    decide(gps,compass,lane,speed,obstacle,reasoningStart,outboxPublisher, actionsPublisher, reasoningRatePublisher)

# TODO: Finish this method    
def extractPerceptions(perceptionString):
    gps = 0
    compass = 0
    lane = 0
    speed = 0
    obstacle = 0
    return (gps,compass,lane,speed,obstacle)

# TODO: Finish this method
def decide(gps,compass,lane,speed,obstacle,reasoningStart,outboxPublisher,actionsPublisher,reasoningRatePublisher):
    # Think about what to do
    
    action = "cat(meow)"
    act(action,actionsPublisher)
    sendMessage(action,outboxPublisher)
    sendReasoningPerforamnce(reasoningStart, datetime.now(), reasoningRatePublisher)
    
    
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
    publisher.pub(str(reasoningTime))


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
