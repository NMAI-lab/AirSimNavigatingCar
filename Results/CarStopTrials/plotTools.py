# -*- coding: utf-8 -*-
"""
Created on Thu Oct 14 13:51:42 2021

@author: Patrick
"""

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os
from datetime import timedelta
import matplotlib.ticker as mtick
import numpy as np

resultIndex = ['agentType', 'obstacleDecisionRange', 'crash', 'miss', 'stop', 'decisionTime', 
               'stopTime', 'distance', 'perceptionCount', 'perceptionPeriod',
               'reasoningPeriod']


def getCrashCounts(resultTuples):

    global resultIndex
    crashCounts = {}
    totals = {}
    
    for row in resultTuples:
        agentType = row[resultIndex.index('agentType')]
        obstacleDecisionRange = float(row[resultIndex.index('obstacleDecisionRange')])
        crash = row[resultIndex.index('crash')]
        
        # Outer dict for the agent type. Inner dict for the ranges. Counts are inner most
        
        # Agent type level
        if agentType != 'reactive':
            if not agentType in crashCounts.keys():
                crashCounts[agentType] = {}
                totals[agentType] = {}
        
            # Stop range level
            if not obstacleDecisionRange in crashCounts[agentType].keys():
                crashCounts[agentType][obstacleDecisionRange] = 0
                totals[agentType][obstacleDecisionRange] = 0
            
            # Increment the crash count if there was a crash
            totals[agentType][obstacleDecisionRange] += 1
            if crash:
                crashCounts[agentType][obstacleDecisionRange] += 1
            
    for agentType in crashCounts.keys():
        for decisionPoint in crashCounts[agentType].keys():
            crashCounts[agentType][decisionPoint] = crashCounts[agentType][decisionPoint] / totals[agentType][obstacleDecisionRange] * 100
            
    return crashCounts


def getPeriods(resultTuples, perception, minimum, maximum):
    periods = {}
    perceptionKey = 'Perception'
    
    for row in resultTuples:
        if perception:
            currentPeriods = row[resultIndex.index('perceptionPeriod')]    
            key = perceptionKey
        else:
            key = row[resultIndex.index('agentType')]
            currentPeriods = row[resultIndex.index('reasoningPeriod')]
            
        if not key in periods.keys():
            periods[key] = {}

        for value in currentPeriods:
            if isinstance(value,timedelta):
                valueNum = value.total_seconds()
                if (valueNum > minimum) and (valueNum < maximum):
                    i = len(periods[key].keys())
                    periods[key][i] = valueNum
                
    return periods


# How long did it take for the agent to decide to stop?
def getParameterData(resultTuples, parameter, limits, useReactive, removeCrashes, removeMisses):
    parameterData = {}
    (minimum, maximum) = limits
    removed = list()
    
    for row in resultTuples:
        agentType = row[resultIndex.index('agentType')]
        parameterInstance = row[resultIndex.index(parameter)]

        crash = row[resultIndex.index('crash')]
        miss = row[resultIndex.index('miss')]
        stop = row[resultIndex.index('stop')]
        reactive = (agentType == 'reactive')
        
        if removeCrashes and crash:
            useThisData = False
        elif removeMisses and miss:
            useThisData = False
        #elif not stop:
        #    useThisData = False
        elif reactive and not useReactive:
            useThisData = False
        else:
            useThisData = True
            
        # Outer dict for the agent type. Inner dict for the ranges. Parameter results are inner most
        
        # Agent type level
        if useThisData:
            if not agentType in parameterData.keys():
                parameterData[agentType] = {}

            if isinstance(parameterInstance,timedelta):
                valueNum = parameterInstance.total_seconds()
            else:
                valueNum = parameterInstance
                
            
            if (valueNum >= minimum) and (valueNum <= maximum):
                i = len(parameterData[agentType].keys())
                parameterData[agentType][i] = valueNum
            else:
                removed.append((agentType,valueNum))
        else:
            if isinstance(parameterInstance,timedelta):
                valueNum = parameterInstance.total_seconds()
            else:
                valueNum = parameterInstance
            removed.append((agentType,valueNum))
                
    #if ('decisionTime' in parameter):
    #    print("this")
            
    return parameterData, removed
    

def plotBarData(dataDict, xLabel, yLabel, fileName):
    dataFrame = pd.DataFrame(dataDict)
    dataFrame = dataFrame.sort_index(axis = 'index')
    ax = dataFrame.plot.bar(ylabel = yLabel, xlabel = xLabel)
    ax.yaxis.set_major_formatter(mtick.PercentFormatter())
    plt.grid()
 
    dirName = 'plots'
    if not os.path.exists(dirName):
        os.makedirs(dirName)
    plt.savefig(dirName + '/' + fileName + '.jpg', format='jpg', dpi=1000,
                bbox_inches='tight')
    plt.clf()


def plotViolinDataSns(dataDict, xLabel, yLabel, fileName):
    dataFrame = pd.DataFrame(dataDict)
    dataFrame = dataFrame.sort_index(axis = 'columns')
    
    ax = sns.violinplot(data=dataFrame)
    ax.set_xlabel(xLabel)
    ax.set_ylabel(yLabel)
    plt.grid()
    
    dirName = 'plots'
    if not os.path.exists(dirName):
        os.makedirs(dirName)
    plt.savefig(dirName + '/' + fileName + '.jpg', format='jpg', dpi=1000,
                bbox_inches='tight')
    plt.clf()
    
def plotViolinData(dataDict, xLabel, yLabel, fileName):
       
    plotData = []
    labels = []
    for scenario in dataDict.keys():
        scenarioData = []
        for i in dataDict[scenario].keys():
            scenarioData.append(dataDict[scenario][i])
        plotData.append(np.array(scenarioData))
        labels.append(scenario)
        
    # Create a figure instance
    fig = plt.figure()

    # Create an axes instance
    ax = fig.add_axes([0,0,1,1])

    # Create the boxplot
    #ax.violinplot(plotData)
    ax.boxplot(plotData)
    
    ax.xaxis.set_tick_params(direction='out')
    ax.xaxis.set_ticks_position('bottom')
    ax.set_xticks(np.arange(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_xlim(0.25, len(labels) + 0.75)
    
    ax.set_xlabel(xLabel)
    ax.set_ylabel(yLabel)
    plt.grid()
    
    dirName = 'plots'
    if not os.path.exists(dirName):
        os.makedirs(dirName)
    plt.savefig(dirName + '/' + fileName + '.jpg', format='jpg', dpi=1000,
                bbox_inches='tight')
    plt.clf()
    
    
def plotCrashes(resultTuples):
    crashCounts = getCrashCounts(resultTuples)
    plotBarData(crashCounts, 'Decision Point (m)', '% Crashes', 'crashes')
    
# Decision time, stop time, stop distance
def plotStopPerformance(resultTuples):
    parameters = ['decisionTime', 'stopTime', 'distance']
    limits = {'decisionTime':(0.001,1),
              'stopTime':(0.001,10),
              'distance':(0.001,10)}
    useReactive = False
    removeCrashes = False#True
    removeMisses = False#True
    for parameter in parameters:
        (parameterData,_) = getParameterData(resultTuples, parameter, limits[parameter], useReactive, removeCrashes, removeMisses)
    
        if ('Time' in parameter) or ('Period' in parameter):
            unit = 's'
        else:
            unit = 'm'
            
        data = parameterData
        xLabel = 'Agent Type'
        yLabel = parameter + ' (' + unit + ')'
        fileName = parameter
        plotViolinData(data, xLabel, yLabel, fileName)  

def plotReasoningPerformance(resultTuples):
    
    data = getPeriods(resultTuples, True, 0, 0.15)
    xLabel = ''
    yLabel = 'Perception Period (s)'
    fileName = 'PerceptionPeriod'
    plotViolinData(data, xLabel, yLabel, fileName) 
    
    data = getPeriods(resultTuples, False, 0, 0.15)
    xLabel = 'Agent Type'
    yLabel = 'Reasoning Time (s)'
    fileName = 'ReasoningTime'
    plotViolinData(data, xLabel, yLabel, fileName) 
    
    
def visualizeResult(resultTuples):
    plotCrashes(resultTuples)
    plotStopPerformance(resultTuples)
    plotReasoningPerformance(resultTuples)
    
    # Plot relative to perception noise (AM I DOING THIS?)
    
    
def test():
    d = {'A': {'apples': 3,
               'bananas':5,
               'oranges':6,
               'kiwis':9},
         'B': {'apples': 1,
               'bananas':9,
               'oranges':3,
               'kiwis':1},
         'C': {'apples': 6,
               'bananas':9,
               'oranges':3,
               'kiwis':3}}
    
    plotBarData(d,'fruit', 'num', 'barDemo')
    
    e = {'Alpha':{'a':3,
                  'b':5,
                  'c':8,
                  'd':5},
        'Bravo':{'a':6,
                 'b':10,
                 'c':16,
                 'd':10}}
    plotViolinData(e, 'Values', 'Results', 'violinDemo')

    
'''
Main program function call - for testing
'''
if __name__ == '__main__':
    test()