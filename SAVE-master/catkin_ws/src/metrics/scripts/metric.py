import numpy as np

class Metric(object):

    def __init__(self, data_size):
        self.data_size = data_size
        self.data = []

    def addElement(self, element):
        if element != None:
            if len(self.data) < self.data_size:
                self.data.insert(0, element)
            else:
                self.data.pop()
                self.data.insert(0, element)

    def getStats(self):
        ret = {}
        if (len(self.data) > 0):
            ret = {
                "avg": np.average(self.data),
                "var": np.var(self.data),
                "min": np.min(self.data),
                "max": np.max(self.data)
            }
        return ret