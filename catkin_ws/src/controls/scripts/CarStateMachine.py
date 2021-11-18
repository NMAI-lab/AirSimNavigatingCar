# -*- coding: utf-8 -*-
"""
Created on Thu Nov 18 10:18:24 2021

@author: Patrick
"""

from stateMachine import StateMachine

class CarStateMachine:
        
    def __init__(self):
        self.stateMachine = StateMachine()
        self.loadStates()
        self.nextWaypoint()
        self.destination()
        
    # TODO
    def loadStates(self):
        self.stateMachine.addStateTransition('Speed0', 'waypointFar', 'setSpeed(8)', 'SpeedFull')
        
    def update(self, perception):
        (gps, lka, speed, compass, obstacle) = perception
        trigger = self.processPerception(gps,lka,obstacle)
        rawAction = self.stateMachine.updateState(trigger)
        action = self.processAction(rawAction,lka,compass,gps)
        return action
    
    
    # TODO
    # Need to generate a state machine trigger from the perception information
    def processPerception(self, gps, lka, obstacle):
        
        procesedPerception = 0
    
        
        # trigger outputs:
        # waypoint: at, near, far
        # lka: available, not available
        # obstacle: near, far
    
        return procesedPerception
    
    
    
    # TODO
    def getPositionTrigger(gps):
        return 'meow'
        
    # TODO
    def getLkaTrigger(lka):
        return 'meow'
    
    # TODO
    def getObstacleTrigger(obstacle):
        return 'meow'
        
    # TODO
    def processAction(self, action, lka, compass, gps):
        processedAction = 0
        return processedAction
    
    # TODO
    def getCompassSteeringAction(compass,gps):
        return 0
    
    # TODO
    def getLkaSteeringAction(lka):
        return 0