# -*- coding: utf-8 -*-
"""
Analysis of the data from the car stop trials
"""

from parsingTools import getData
from plotTools import visualizeResult



def performAnalysis(useCsv = False, resultFile = 'results.csv'):
    resultTuples = getData(useCsv, resultFile) 
    
    paperVersion = True
    if paperVersion:
        for i in range(len(resultTuples)):
            item = resultTuples[i]
            if 'AIB' in item:
                itemList = list(item)
                itemList[0] = 'idiomatic'
                item = tuple(itemList)
                resultTuples[i] = item
    
    visualizeResult(resultTuples)
      
    
'''
Main program function call
'''
if __name__ == '__main__':
    performAnalysis(useCsv = True)