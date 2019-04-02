import agentThread
import time
from time import sleep
from gvh import gvh
from threading import Lock
import dsm


class TestApp1(agentThread.agentThread):

    def __init__(self, name, bcastqueue, barrier, dd, numbots=1, locks=[]):
        super(TestApp1, self).__init__(gvh(name, bcastqueue, numbots), dd, locks)
        self.barrier = barrier

    def run(self):

        pid = self.id
        numBots = self.gvh.participants
        lock0 = self.locks[0]
        sum = self.dd.get('sum')
        if sum is None:
            sum = dsm.dsmvar('sum', -1, 0, ('int', 'aw'), -1)
            self.dd.add(sum)

        numadded = self.dd.get('numadded')
        if numadded is None:
            numadded = dsm.dsmvar('numadded', -1, 0, ('int', 'aw'), -1)
            self.dd.add(numadded)

        sum = 0
        numadded = 0
        added = False
        finalsum = None
        dummy = 0
        while (not (self.stopped())):

            sleep(1)  # adding
            if (not (added)):

                with lock0:

                    if (self.dd.get("sum") == None):
                        continue

                    sum = self.dd.get("sum")
                    sum = (sum + (pid * 2))
                    self.dd.put("sum", sum, time.time())
                    if (self.dd.get("numadded") == None):
                        continue

                    numadded = self.dd.get("numadded")
                    numadded = (numadded + 1)
                    self.dd.put("numadded", numadded, time.time())
                    added = True

            # dummyadd
            if (self.dd.get("numadded") == None):
                continue

            numadded = self.dd.get("numadded")
            if ((numadded < numBots)):

                if (self.dd.get("sum") == None):
                    continue

                sum = self.dd.get("sum")
                dummy = (dummy + sum)
                print("pid:", pid, "dummy:", dummy)

            # finalsum
            if (self.dd.get("numadded") == None):
                continue

            numadded = self.dd.get("numadded")
            if ((numadded == numBots)):

                if (self.dd.get("sum") == None):
                    continue

                sum = self.dd.get("sum")
                finalsum = sum
                print("finalsum:",finalsum)
                self.stop()









