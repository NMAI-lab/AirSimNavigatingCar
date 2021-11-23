# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 15:52:06 2021

@author: Patrick
"""

class StateMachine:
    
    def __init__(self):
        self.state = ''
        self.stateTransitions = {}
        self.triggerActionParameterCount = {}

    def addStateTransition(self, startState, trigger, action, newState):
        if not startState in self.stateTransitions.keys():
            self.stateTransitions[startState] = {}
        self.stateTransitions[startState][trigger] = (action,newState)
    
    def updateState(self, trigger):
        startState = self.state
        (action,self.state) = self.stateTransitions[startState][trigger]
        
        return action
    
    def setState(self, newState):
        if newState in self.stateTransitions.keys():
            self.state = newState
            return True
        else:
            print("Unknown state provided, ignored.")
            return False
        

def buildStateMachine():
    machine = StateMachine()
    machine.state = 'a'
    machine.addStateTransition('a', 1, 'do(1)', 'b')
    machine.addStateTransition('a', 2, 'do(2)', 'a')
    machine.addStateTransition('b', 1, 'do(1)', 'a')
    machine.addStateTransition('b', 2, 'do(2)', 'b')
    return machine
    
    
def testStateTransition(machine, trigger):
    print("In state: " + str(machine.state) + ", trigger: " + str(trigger))
    action = machine.updateState(trigger)
    print("Action: " + str(action) + ", new state: " + str(machine.state))
    
    
def testStateMachine():
    machine = buildStateMachine()
    testStateTransition(machine, 1)
    testStateTransition(machine, 1)
    testStateTransition(machine, 2)
    testStateTransition(machine, 2)

if __name__ == '__main__':
    testStateMachine()