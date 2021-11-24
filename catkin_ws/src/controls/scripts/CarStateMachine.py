# -*- coding: utf-8 -*-
"""
Created on Thu Nov 18 10:18:24 2021

@author: Patrick
"""

from stateMachine import StateMachine
import nvector as nv

class CarStateMachine:
        
    def __init__(self):
        self.stateMachine = StateMachine()
        self.loadStates()
        
        self.declanation = 7.5
        self.nearWaypointRange = 20
        self.atWayPointRange = 7
        self.obstacleRange = 5
        
        self.wgs84 = nv.FrameE(name='WGS84')
        self.waypoint = self.wgs84.GeoPoint(latitude=6426242556, longitude=-122.140354517, degrees = True)
        
        self.stateMachine.setState('SpeedSet0')
        

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
        waypointRange = self.waypoint.delta_to(current).length
        
        position = ""
        if waypointRange < self.atWayPointRange:
            position = "at"
        elif waypointRange < self.nearWaypointRange:
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
        
    def getCurrentState(self):
        return self.stateMachine.state
        
    def processAction(self, action, lka, compass, gps):
        if "lkaSteer" in action:
            processedAction = "steering(" + str(self.getLkaSteeringAction(lka)) + ")"
        elif "compassSteer" in action:
            processedAction = "steering(" + str(self.getCompassSteering(gps, compass)) + ")"
        elif "none" in action:
            processedAction = ""
        else:
            processedAction = action

        return processedAction
    
    
    def getCompassSteering(self, gps, compass):
        (curLat,curLon) = gps
        current = self.wgs84.GeoPoint(latitude=curLat, longitude=curLon, degrees = True)
        waypointBearing = self.waypoint.delta_to(current).azimuth_deg
        courseCorrection = waypointBearing - (compass + self.declanation)
        
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
    
def testCarStateMachine():
    testMachine = CarStateMachine()
    print("Start State: " + testMachine.getCurrentState())
    
    # At starting location. Car should set speed to full
    gps = (47.64148237134305, -122.14036499083176)
    compass = -5.63447277411
    speed = 0.0
    lka = (0.0,0.0,0.0,0.0,0.0)
    obstacle = 48.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState()) 
    
    # Car is driving. No LKA, should do compass steering
    gps = (47.64148237134305, -122.14036499083176)
    compass = -5.63447277411
    speed = 8.0
    lka = (0.0,0.0,0.0,0.0,0.0)
    obstacle = 48.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState())     
    
    
    # Car is driving. LKA available, should do LKA steering
    gps = (47.64148237134305, -122.14036499083176)
    compass = -80
    speed = 8.0
    lka = (0.6118902439024382,0.0,0.0,0.30147058823529455,84.41176470588215)
    obstacle = 48.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState()) 
    
    
    # Car is driving. LKA available, obstacle is in range, should do avoidance
    gps = (47.64148237134305, -122.14036499083176)
    compass = -80
    speed = 8.0
    lka = (0.6118902439024382,0.0,0.0,0.30147058823529455,84.41176470588215)
    obstacle = 4.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState()) 
    
    # Car is driving. LKA not available, obstacle is in range, should do avoidance
    gps = (47.64148237134305, -122.14036499083176)
    compass = -80
    speed = 8.0
    lka = (0.0,0.0,0.0,0.0,0.0)
    obstacle = 4.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState()) 
    
    # Car is driving. LKA available, near waypoint, should slow down
    gps = (47.64148237134305, -122.14036499083176)
    compass = -80
    speed = 8.0
    lka = (0.0,0.0,0.0,0.0,0.0)
    obstacle = 4.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState()) 
    
    # TODO
    # Car is driving. LKA available, near waypoint, should slow down
    gps = (47.64148237134305, -122.14036499083176) # Figure our the coordinates
    compass = -80
    speed = 8.0
    lka = (0.0,0.0,0.0,0.0,0.0)
    obstacle = 4.2278885841
    perception = (gps, lka, speed, compass, obstacle)
    action = testMachine.update(perception)
    print("Action: " + action + ", New State: " + testMachine.getCurrentState()) 
    
if __name__ == '__main__':
    testCarStateMachine()
    