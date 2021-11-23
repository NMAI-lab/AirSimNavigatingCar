# -*- coding: utf-8 -*-
"""
Created on Thu Nov 18 10:18:24 2021

@author: Patrick
"""

from stateMachine import StateMachine
import nvframe as nv

class CarStateMachine:
        
    def __init__(self):
        self.stateMachine = StateMachine()
        self.loadStates()
        self.nextWaypoint()
        self.destination()
        
        self.declanation = 7.5
        self.nearWaypointRange = 20
        self.atWayPointRange = 7
        self.obstacleRange = 5
        
        self.wgs84 = nv.FrameE(name='WGS84')
        self.destination = self.wgs84.GeoPoint(latitude=6426242556, longitude=-122.140354517, degrees = True)
        
    # TODO
    def loadStates(self):
        self.stateMachine.addStateTransition('SpeedSet0',       ('at',      False,  False),     'none',         'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSet0',       ('at',      False,  True),      'none',         'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSet0',       ('at',      True,   False),     'none',         'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSet0',       ('at',      True,   True),      'none',         'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSet0',       ('near',    False,  False),     'setSpeed(3)',  'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSet0',       ('near',    False,  True),      'setSpeed(3)',  'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSet0',       ('near',    True,   False),     'setSpeed(3)',  'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSet0',       ('near',    True,	True),      'setSpeed(3)',  'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSet0',       ('far',     False,	False),     'setSpeed(8)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSet0',       ('far',     False,  True),      'setSpeed(8)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSet0',       ('far',     True,   False),     'setSpeed(8)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSet0',       ('far',     True,   True),      'setSpeed(8)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('at',      False,  False),     'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('at',      False,  True),      'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('at',      True,   False),	    'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('at',      True,   True),      'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('near',    False,  False),     'setSpeed(3)',	'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('near',    False,  True),      'steer(-0.3)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('near',    True,   False),     'setSpeed(3)',	'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('near',    True,   True),      'steer(-0.3)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('far',     False,  False),     'compassSteer',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('far',     False,  True),      'steer(-0.3)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('far',     True,   False),     'lkaSteer',     'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetFull',    ('far',     True,   True),      'steer(-0.3)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('at',      False,  False),     'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('at',      False,  True),      'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('at',      True,   False),     'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('at',      True,   True),      'setSpeed(0)',	'SpeedSet0')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('near',    False,  False),     'compassSteer',	'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('near',    False,  True),      'steer(-0.3)',	'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('near',    True,   False),     'lkaSteer',	    'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('near',    True,   True),      'steer(-0.3)',	'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('far',     False,  False),     'setSpeed(8)',	'SpeedSetFull')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('far',     False,  True),      'steer(-0.3)',	'SpeedSetSlow')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('far',     True,   False),     'setSpeed(8)',	'SetSpeedFull')
        self.stateMachine.addStateTransition('SpeedSetSlow',    ('far',     True,   True),      'steer(-0.3)',	'SpeedSetFull')
        
        
    def update(self, perception):
        (gps, lka, speed, compass, obstacle) = perception
        trigger = self.processPerception(gps,lka,obstacle)
        rawAction = self.stateMachine.updateState(trigger)
        action = self.processAction(rawAction,lka,compass,gps)
        return action
    
    
    # Need to generate a state machine trigger from the perception information
    def processPerception(self, gps, lka, obstacle):
        positionTrigger = self.getPositionTrigger(gps)
        lkaTrigger = self.getLkaTrigger(lka)
        obstacleTrigger = self.getObstacleTrigger(obstacle)
        return (positionTrigger, lkaTrigger, obstacleTrigger)
    
    
    # waypoint: at, near, far
    def getPositionTrigger(self, gps):
        (curLat,curLon) = gps
        current = self.wgs84.GeoPoint(latitude=curLat, longitude=curLon, degrees = True)
        destinationRange = self.destination.delta_to(current).length
        
        position = ""
        if destinationRange < self.atWayPointRange:
            position = "at"
        elif destinationRange < self.nearWaypointRange:
            position = "near"
        else:
            position = "far"
        
        return position
    
    
    # lka available: True, False
    def getLkaTrigger(self, lka):
        (_,_,_,c,d) = lka
        if ((c != 0) or (d != 0)):
            return True
        else:
            return False
    
    # obstacle near: True, False
    def getObstacleTrigger(self, obstacle):
        if obstacle <= self.obstacleRange:
            return True
        else:
            return False
        
    def processAction(self, action, lka, compass, gps):
        if "lkaSteering" in action:
            processedAction = "steering(" + str(self.getLkaSteeringAction(lka)) + ")"
        elif "compassSteering" in action:
            processedAction = "steering(" + str(self.getCompassSteering(gps, compass)) + ")"
        else:
            processedAction = action

        return processedAction
    
    
    def getCompassSteering(self, gps, compass):
        (curLat,curLon) = gps
        current = self.wgs84.GeoPoint(latitude=curLat, longitude=curLon, degrees = True)
        destinationBearing = self.destination.delta_to(current).azimuth_deg[0]
        courseCorrection = destinationBearing - (compass + self.declanation)
        
        if courseCorrection >= 20: 
            steeringSetting = 1
        elif courseCorrection <= -20: 
            steeringSetting = -1
        else: 
            steeringSetting = courseCorrection/180
            
        return steeringSetting
    

    def getLkaSteeringAction(self, lka):
        (steering,_,_,_,_) = lka
        return steering