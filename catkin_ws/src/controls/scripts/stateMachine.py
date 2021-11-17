# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 15:52:06 2021

@author: Patrick
"""

state = ''
stateTransitions = {}

def addStateTransition(startState, trigger, action, newState):
    global stateTransitions
    if not startState in stateTransitions.keys():
        stateTransitions[startState] = {}
    stateTransitions[startState][trigger] = (action,newState)
    
def updateState(trigger):
    global state, stateTransitions
    startState = state
    (action,state) = stateTransitions[startState][trigger]
    return action

def buildStateMachine():
    global state
    state = 'a'
    addStateTransition('a', 1, 'do(1)', 'b')
    addStateTransition('a', 2, 'do(2)', 'a')
    addStateTransition('b', 1, 'do(1)', 'a')
    addStateTransition('b', 2, 'do(2)', 'b')
    
    
def testStateTransition(trigger):
    global state
    print("In state: " + str(state) + ", trigger: " + str(trigger))
    action = updateState(trigger)
    print("Action: " + str(action) + ", new state: " + str(state))
    
    
def testStateMachine():
    buildStateMachine()
    testStateTransition(1)
    testStateTransition(1)
    testStateTransition(2)
    testStateTransition(2)

if __name__ == '__main__':
    testStateMachine()