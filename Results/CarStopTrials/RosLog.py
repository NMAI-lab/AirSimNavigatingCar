# -*- coding: utf-8 -*-
"""
Created on Tue Sep 21 15:54:02 2021

@author: Patrick
"""

import csv
from os import listdir
import os
import re
import dateutil.parser
from datetime import datetime
from datetime import timedelta
import nvector as nv

class RosLog:
    
    def __init__(self, path):
        
        fileNames = self.getPathsToLogs(path)
        topic = list()
        message = list()
        timeStamp = list()
        

        self.agentType = path.split('_')[0].split('/')[1]
        
        self.obstacleRange = path.split('_')[1]
        self.obstacleRange = float(self.obstacleRange.split('m')[0])
        
        
        self.outcome = ('crash' in path, 'miss' in path, 'stop' in path)
       
        
        self.reasoningPerformance = list()
        self.perceptions = list()
        self.outbox = list()
        self.inbox = list()
        self.actions = list()        
        
        for file in fileNames:
            (topic,message,timeStamp) = self.parseFile(file)
            for i in range(len(topic)):
                if "reasoningPerformance" in topic[i]:
                    if message[i].isdigit():
                        self.reasoningPerformance.append((timeStamp[i],timedelta(milliseconds = int(message[i]))))
                    else:
                        start = dateutil.parser.parse("0:0")
                        stop = dateutil.parser.parse(message[i])
                        delta = stop - start
                        self.reasoningPerformance.append((timeStamp[i],delta))
                elif "perception" in topic[i]:
                    self.perceptions.append((timeStamp[i],message[i]))
                elif "outbox" in topic[i]:
                    self.outbox.append((timeStamp[i],message[i]))
                elif "inbox" in topic[i]:
                    self.inbox.append((timeStamp[i],message[i]))
                elif "action" in topic[i]:
                    self.actions.append((timeStamp[i],message[i]))
        

    def getOutcome(self):
        return self.outcome

    def getAgentType(self):
        return self.agentType
    
    def getObstacleRange(self):
        return self.obstacleRange
        
    '''
    Use this directory to target the analysis to specific test logs, groups of test
    logs or all the logs
    '''
    def getPathsToLogs(self, startPath = '.'):
    
        csvFiles = list()
    
        # Get all the directories. Some may not have CSV files in them
        directories = [x[0] for x in os.walk(startPath)]
    
        for directory in directories:
            files = self.findCsvFilenames(path=directory, suffix=".csv")
            for file in files:
                csvFiles.append(directory + "\\" + file)
    
        return csvFiles    
        
    '''
    Parse a given csv file
    '''
    def parseFile(self, fileName):
        with open(fileName) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
    
            topic = list()
            message = list()
            timeStamp = list()

            for row in csv_reader:
                if len(row) > 0:
                    currentTopic = row[0]
                    currentMessage = row[1]
                    currentTimeStamp = row[2]

                    if ((len(currentTopic) > 0) and (not "Topic" in currentTopic)):
                        topic.append(currentTopic)
                        message.append(currentMessage)
                        timeStamp.append(currentTimeStamp)            
            
            return (topic, message, timeStamp)
        
    '''
    Get a list of the file names in a given directory
    '''
    def findCsvFilenames(self, path=".", suffix=".csv"):
        filenames = listdir(path)
        return [ filename for filename in filenames if filename.endswith( suffix ) ]
    
    def buildTimeline(self):
        
        self.times = dict()
        self.times['start'] = 0
        startTime = self.inbox[0][0]
        
        milestones = ['gettingRoute', 'route', 'arrived']
        
        for event in self.outbox:
            for milestone in milestones:
                if milestone in event[1]:
                    self.times[milestone] = getTimeDelta(startTime,event[0])
                elif (('move' in event[1]) or ('controlSpeed' in event[1])) and not ('move' in self.times.keys()):
                    self.times['move'] = getTimeDelta(startTime,event[0])

    
    def calculatePlanUsage(self):
        self.planUsage = dict()
       
        for instance in self.outbox:
            content = instance[1].split('BROADCAST,')[1]
            content = content.split('>')[0]
            planName = content.split('(')[0]
            
            if 'default' in content:
                planName = planName + 'Default'
            if planName in self.planUsage.keys():
                self.planUsage[planName] += 1
            else:
                self.planUsage[planName] = 1
                
    def calculateActionUsage(self):
        self.actionUsage = dict()
        
        actionsOfInterest = ['steering', 'setSpeed', 'getPath', 'move']
        messageOfInterest = ['navigate(gettingRoute']
        
        for instance in self.actions:
            for actionName in actionsOfInterest:
                content = instance[1]
                if actionName in content:
                    if actionName in self.actionUsage.keys():
                        self.actionUsage[actionName] += 1
                    else:
                        self.actionUsage[actionName] = 1
                       
        # Check for the case where internal actions were used. If so, 
        # we will not have seen any getPath used yet
        if not 'getPath' in self.actionUsage.keys():
            for instance in self.outbox:
                for actionFlag in messageOfInterest:
                    content = instance[1]
                    if actionFlag in content:
                        if 'getPath' in self.actionUsage.keys():
                            self.actionUsage['getPath'] += 1
                        else:
                            self.actionUsage['getPath'] = 1

    def getMertics(self):
        return (self.getReasoningRate(), self.getActionUsage(), self.getPlanUsage(), self.getTimeLine())
    
    # Get how long it took for the agent to react to a collision perception
    def getObstacleActionTime(self):
        (start,dist) = self.getObstacleTime()
        stopAction = self.getStopActionTime()
        stop = self.getStopTime(stopAction)
        
        startPosition = self.getPositionAtTime(start)
        stopPosition = self.getPositionAtTime(stop)
        
        if stopPosition != -1:
            delta = startPosition.delta_to(stopPosition)
            distance = delta.length[0]
        else:
            distance = -1
        
        if (start != -1) and (stopAction != -1):
            decisionTime = stopAction - start
        else:
            decisionTime = -1
            
        if (start != -1) and (stop != -1):
            stopTime = stop - start
        else:
            stopTime = -1
            
        return (decisionTime, stopTime, distance)
        
        
    # Get the time when the car is stopped. Start search at timeStep 'after'
    def getStopTime(self, after):
        for percept in self.perceptions:
            perceptStrings = percept[1].split(' ')
            for ps in perceptStrings:
                if 'speed' in ps:
                    observedSpeed = float(re.search('\((.*)\)', ps).group(1))
                    if observedSpeed < 0.01:    # Account for sensor noise (it is rarely exactly 0)
                        time = dateutil.parser.parse(percept[0])
                        if (type(time) == datetime) and (type(after) == datetime):
                            if time > after:
                                return time
        
        # Default case, we didn't find anything
        return -1
        

    # Get the time when the obstacle was in the stopping range    
    def getObstacleTime(self):
        for percept in self.perceptions:
            perceptStrings = percept[1].split(' ')
            for ps in perceptStrings:
                if 'obstacle' in ps:
                    observedObstacleRange = float(re.search('\((.*)\)', ps).group(1))
                    if observedObstacleRange < self.obstacleRange:
                        time = dateutil.parser.parse(percept[0])
                        return (time,observedObstacleRange)
        
        return(-1,-1)  # If we didn't find anything
    
    def getPositionAtTime(self, time):
        for percept in self.perceptions:
            perceptTime = dateutil.parser.parse(percept[0])
            if perceptTime == time:
                perceptStrings = percept[1].split(' ')
                for i in range(len(perceptStrings)):
                    if 'gps' in perceptStrings[i]:
                        lat = float(perceptStrings[i].split('(')[1].split(',')[0])
                        lon = float(perceptStrings[i+1].split(')')[0])
                        wgs84 = nv.FrameE(name='WGS84')
                        locationCoord = wgs84.GeoPoint(latitude=lat, longitude=lon, degrees = True)
                        return locationCoord
        
        return -1
    
    
    # Find the time when the stop action was commanded
    def getStopActionTime(self):
        for action in self.actions:
            actionString = action[1]
            if 'setSpeed(0)' in actionString:
                return dateutil.parser.parse(action[0])
           
        return -1
            
    
    def getTimeLine(self):
        return self.times.copy()
    
    def getReasoningRate(self):
        return [int(data[1]) for data in self.reasoningPerformance]
    
    def getPlanUsage(self):
        return self.planUsage.copy()
    
    def getActionUsage(self):
        return self.actionUsage.copy()
    
    
    def getPerceptPeriod(self):
        periodList = list()
        lastTime = -1
        for percept in self.perceptions:
            currentTime = dateutil.parser.parse(percept[0])
            if lastTime != -1:
                periodList.append(currentTime-lastTime)
                lastTime = currentTime
            else:
                lastTime = currentTime
        return periodList
    

    def getReasoningPeriod(self):
        periodList = list()
        for rp in self.reasoningPerformance:
            periodList.append(rp[1])
        return periodList
    
    def getPerceptionTypes(self):
        perceptionTypes = list()
        sample = self.perceptions[0][1]
        segments = sample.split(' ')
        for s in segments:
            if '(' in s:
                perceptionTypes.append(s.split('(')[0])
        return perceptionTypes
    

def getTimeDelta(a, b):
    if not '.' in a:
        a += '.0'
    if not '.' in b:
        b += '.0'
    
    aTime = dateutil.parser.parse(a)
    bTime = dateutil.parser.parse(b)
    delta = bTime - aTime   
    
    # Return in ms
    return delta.total_seconds() * 1000
   
 
'''
test case
'''
def test():
    #log = RosLog('Logs/AirSimNavigatingCar/AgentSpeak/1')
    log = RosLog('Logs/JasonMobileAgentRos/EnvironmentSupport/2')
    (periodList, actionUsage, planUsage, timeline) = log.getMertics()
        
'''
Main program function call
'''
if __name__ == '__main__':
    test()