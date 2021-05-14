#!/usr/bin/env python

# Simple test tool for the user interface.

# @author: Patrick Gavigan

import rospy
from std_msgs.msg import String
import time
from datetime import datetime

startTime = 0   # Set the start time as a global variable

def sendMailMission(publisher):
    messageID = int(round(time.time() * 1000))  # Crude message ID
    agentID = "BROADCAST"
    userID = "user"
    messageType = "tell"

    # Prompt user for input (using Pyton 2 method, ROS does not use Python 3)
    destination = raw_input("Please enter the destination location (Example: post3 or d): ")

    # Build message with the mail mission, log and send it
    messageType = "achieve"
    messageContent = "mission(navigate,[" + str(destination) + "])"  
    message = "<" + str(messageID) + "," + userID + "," + messageType + "," + agentID + "," + messageContent + ">"
    rospy.loginfo("Sending message: " + str(message))
    publisher.publish(message)


# Receive outbox messages. Just print everything.
def receiveMessage(data):
    rospy.loginfo("Received message: " + str(data.data))
    
    # Check if the message contains the delivered signal
    if "mailUpdate(delivered)" in data.data:
        global startTime
        endTime = datetime.now()
        rospy.loginfo("**** Mission complete ****")
        rospy.loginfo("Start time was: " + str(startTime))
        rospy.loginfo("End time was: " + str(endTime))
        

# Main program
def rosMain():

    # Init the node
    rospy.init_node('user', anonymous=True)

    # Subscribe to outbox
    rospy.Subscriber('outbox', String, receiveMessage)

    # Setup the publisher for the result
    publisher = rospy.Publisher('inbox', String, queue_size=10)

    # Sleep for 5 seconds, give everything a chance to come online.
    time.sleep(5)

    # Prompt the use for the mailMission and send it
    sendMailMission(publisher)

    # Log the start time of the mission
    global startTime
    startTime = datetime.now()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
        