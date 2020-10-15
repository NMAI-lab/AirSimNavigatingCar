#!/usr/bin/env python

# Main file for the navigator

# @author: Patrick Gavigan

import rospy
from RouteSearcher import RouteSearcher
from std_msgs.msg import String

current = (0,0)
previous = (0,0)

# Send the direction update
def sendDirection(data, args):
    (publisher, searcher) = args
    
    # Extract the message
    position = (data.latitude, data.longitude)

    # Get access to the global variables (a bit hacky)
    global previous
    global current
    
    # Check if the post point changed, update history if necessary
    if position != current:
        previous = current
        current = position
    
    # Get the next direction solution    
    solution = searcher.getNextDirection(previous, current)

    # Publish    
    rospy.loginfo("Navigation solution: " + solution)
    publisher.publish(solution)


# Set destination action
def setDestination(data, args):
    (searcher) = args
    destination = data.data
    searcher.setDestination(destination)

# Main program
def rosMain():
    # Setup the searcher
    searcher = RouteSearcher()
    
    # Init the node
    rospy.init_node('navigator', anonymous=True)

    # Subscribe to actions, watch for setDest messages
    rospy.Subscriber('setDestination', String, setDestination, (searcher))

    # Setup the publisher for the result
    publisher = rospy.Publisher('perceptions', String, queue_size=10)
    
    # Listen for post point messages on the perceptions topic
    rospy.Subscriber('postPoint', String, sendDirection, (publisher, searcher))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# Unit tests for the search methods
def unitTest():
    data = "postPoint((47.64197387644888, -122.14039866358259),(47.641973876500984, -122.14039866356809))"
    parameters = data.split(',')
    
    # Setup the searcher
    # searcher = RouteSearcher()
    # searcher.setDestination("post7")
    
    # searcher.setDestination("post2")
    # turn = searcher.getNextDirection("post4", "post3")
    # print("Turn should be left: " + turn)
    
    # searcher.setDestination("post2")
    # turn = searcher.getNextDirection("post5", "post3")
    # print("Turn should be right: " + turn)
    
    # searcher.setDestination("post5")
    # turn = searcher.getNextDirection("post4", "post3")
    # print("Turn should be forward: " + turn)
    
    # searcher.setDestination("post4")
    # turn = searcher.getNextDirection("post5", "post3")
    # print("Turn should be forward: " + turn)
    
    # searcher.setDestination("post3")
    # turn = searcher.getNextDirection("post1", "post2")
    # print("Turn should be left: " + turn)


if __name__ == '__main__':
    try:
        rosMain()
        #unitTest()
    except rospy.ROSInterruptException:
        pass
        