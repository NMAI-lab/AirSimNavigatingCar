#!/usr/bin/env python
# This node is used for logging performance inflormation used for benchmarking performance.

import rospy
from std_msgs.msg import String
from datetime import datetime
import csv
import atexit

logData = {}

def saveLog():
    rospy.loginfo("Saving logs.")
    topics = logData.keys()
    for topic in topics:
        fileName = topic + "Log.csv"
        updateCsv(fileName, 'Topic', 'Data', 'TimeStamp', 'w')
        for item in logData[topic]:
            (content,timeStamp) = item
            updateCsv(fileName, topic, content, timeStamp, "a+")
    rospy.loginfo("Logging complete.")


def updateCsv(fileName, topic, data, timeStamp, m):
    if len(data) > 0:
        with open(fileName, mode=m) as node_log:
            node_logger = csv.writer(node_log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            node_logger.writerow([topic, data, timeStamp])
    

def receiveData(data, topic):
    topic = (topic)
    global logData
        
    # item = (content, timeStamp)
    item = (str(data.data), str(datetime.now()))
    
    if not topic in logData.keys():
        logData[topic] = list()

    logData[topic].append(item)
    

def rosMain():
    atexit.register(saveLog)
    
    rospy.init_node('nodeLogger', anonymous=True)
 
    rospy.Subscriber('actions', String, receiveData, ('actions'))
    rospy.Subscriber('perceptions', String, receiveData, ('perceptions'))
    rospy.Subscriber('reasoningPerformance', String, receiveData, ('reasoningPerformance'))
    rospy.Subscriber('inbox', String, receiveData, ('inbox'))
    rospy.Subscriber('outbox', String, receiveData, ('outbox'))

    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
    
    