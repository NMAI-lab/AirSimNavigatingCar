#!/usr/bin/env python
import threading

# import rospy

class Action:

    def __init__(self):
        self.sem = threading.Semaphore() # Used to ensure add and get action are atomic instructions
        self.actions = {} # Key value pairs to produce an action

    def add(self, key, value):
        self.sem.acquire()
        self.actions[key] = value
        self.sem.release()

    def get(self):
        self.sem.acquire()
        action = self.actions
        self.sem.release()
        return action

    def clear(self):
        self.sem.acquire()
        self.actions = {}
        self.sem.release()

