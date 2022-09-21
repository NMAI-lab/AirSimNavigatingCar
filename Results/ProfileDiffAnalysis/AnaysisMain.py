# -*- coding: utf-8 -*-
"""
Created on Thu Nov 25 18:52:56 2021

@author: Patrick
"""

import xml.etree.ElementTree as ET
import csv


'''
Parse a given csv file
'''
def getCsvData():
    fileName = "data\Hot_spot_comparison_averages.csv"
    with open(fileName) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
    
        data = []
        headdings = []

        for row in csv_reader:
            if len(row) > 0:
                if headdings == []:
                    headdings = row
                else:
                    current = {}
                    for i in range(len(headdings)):
                        current[headdings[i]] = row[i]
                    data.append(current)          
            
        return data

def filterTable(table, filterList):
    filteredTable = []
    for row in table:
        keep = False
        keyList = row.keys()
        for key in keyList:
            if row[key] in filterList:
                keep = True
        if keep:
            filteredTable.append(row)
    return filteredTable
    


def getXmlData():
    hotSpotXmlFile = "data\Call_tree_comparison_averages_reasoningCycleOnly.xml"
    tree = ET.parse(hotSpotXmlFile)
    root = tree.getroot()
    return root


def getMethodNames(root, methodNameList = list()):
    for child in root:
        childTag = child.tag
        if 'node' in childTag:
            childAttribute = child.attrib
            className = childAttribute['class']
            if 'jason' in className:
                methodName = childAttribute['methodName']
                fullName = className + "." + methodName
                if not (fullName in methodNameList):
                    methodNameList.append(fullName)
        methodNameList = getMethodNames(child, methodNameList)
    
    return methodNameList

def main():
    root = getXmlData()
    hotSpots = getCsvData()
    methodList = getMethodNames(root)
    reasoningHotSpots = filterTable(hotSpots, methodList)
    print(reasoningHotSpots)

    
if __name__ == "__main__":
    main()