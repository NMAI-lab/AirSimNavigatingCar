# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 10:48:54 2021

@author: Patrick
"""

import os
import csv
import dateutil.parser
from datetime import timedelta
from RosLog import RosLog


def getData(useCsv, resultFile):
    
    if useCsv and os.path.isfile(resultFile):
        resultTuples = loadCsv(resultFile)
    else: 
        resultTuples = loadRawData()
        exportResults(resultTuples, resultFile)
    return resultTuples


def getPaths(rootPath):
    pathList = os.listdir(path=rootPath)
    fullPaths = list()    
    for path in pathList:
        fullPaths.append(rootPath + path)
    return fullPaths


def exportResults(results, resultFile):
    with open(resultFile,'w',newline='') as out:
        csv_out=csv.writer(out)
        csv_out.writerow(['agentType', 'obstacleDecisionRange', 'crash', 'miss', 'stop', 'decisionTime', 'stopTime', 'distance','perceptionCount','perceptionPeriod','reasoningPeriod'])
        for row in results:
            row = [str(i) for i in row]               
            csv_out.writerow(row)
            

def loadCsv(fileName):
    with open(fileName) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        resultTuples = list()

        next(csv_reader, None)  # Skip the headder
        for row in csv_reader:
            if len(row) > 0:
                rowTuple = convertTypes(row)
                resultTuples.append(rowTuple)
        return resultTuples

def convertTypes(row):
    (agentType, obstacleDecisionRange, crash, miss, stop, decisionTime, stopTime, distance, perceptionCount,perceptionPeriod,reasoningPeriod) = tuple(row)
    
    crash = crash == "True"
    miss = miss  == "True"
    stop = stop == "True"
    
    start = dateutil.parser.parse("0:0")
    finish = dateutil.parser.parse(decisionTime)
    decisionTime = finish - start
    
    finish = dateutil.parser.parse(stopTime)
    stopTime = finish - start
    
    distance = float(distance)
    perceptionCount = int(perceptionCount)
    
    perceptionPeriod = perceptionPeriod.replace("[","")
    perceptionPeriod = perceptionPeriod.replace("]","")
    perceptionPeriod = [timeDeltaFromString(i) for i in perceptionPeriod.split(',')]
    
    reasoningPeriod = reasoningPeriod.replace("[","")
    reasoningPeriod = reasoningPeriod.replace("]","")
    reasoningPeriod = [timeDeltaFromString(i) for i in reasoningPeriod.split(',')]
    
    # Deal with name change (Pyhon agent should be imperative agent)
    if 'python' in agentType:
        agentType = 'imperative'
    
    return (agentType, obstacleDecisionRange, crash, miss, stop, decisionTime, stopTime, distance, perceptionCount,perceptionPeriod,reasoningPeriod)


def timeDeltaFromString(timeString):
    timeString = timeString.replace("datetime.timedelta(","")
    timeString = timeString.replace(")","") 
    
    values = {}
    name, var = timeString.partition("=")[::2]
    values[name.strip()] = float(var)
    delta = timedelta(0)
    for key in values:
        if key == 'days':
            delta = delta + timedelta(days = values[key])
        elif key == 'seconds':
            delta = delta + timedelta(seconds = values[key])
        elif key == 'microseconds':
            delta = delta + timedelta(microseconds = values[key])
        elif key == 'milliseconds':
            delta = delta + timedelta(milliseconds = values[key])
        elif key == 'minutes':
            delta = delta + timedelta(minutes = values[key])
        elif key == 'hours':
            delta = delta + timedelta(hours = values[key])
        elif key == 'weeks':
            delta = delta + timedelta(weeks = values[key])
    return delta
   

def loadRawData():
    pathList = getPaths("data/")
    results = list()
    resultTuples = list()
    for path in pathList:
        
        # Get results
        result = RosLog(path)
        results.append(result)
        (decisionTime, stopTime, distance) = result.getObstacleActionTime()
        agentType = result.getAgentType()
        obstacleDecisionRange = result.getObstacleRange()
        (crash, miss, stop) = result.getOutcome()
        perceptionCount = len(result.getPerceptionTypes())
        perceptionPeriod = result.getPerceptPeriod()
        reasoningPeriod = result.getReasoningPeriod()
        resultTuples.append((agentType, obstacleDecisionRange, crash, miss, stop, decisionTime, stopTime, distance, perceptionCount,perceptionPeriod,reasoningPeriod))
    return resultTuples


def test():
    print("this is a test")
    getData(useCsv = True, resultFile = "results.csv")
    #timeDeltaFromString("datetime.timedelta(microseconds=78000)")


'''
Main program function call - for testing
'''
if __name__ == '__main__':
    test()