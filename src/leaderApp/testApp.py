import agentThread
import time
from time import sleep
from gvh import gvh
from threading import Lock
import dsm


class TestApp(agentThread.agentThread):

    def __init__(self, name, bcastqueue, barrier, dd, numbots=1, locks=[]):
        super(TestApp, self).__init__(gvh(name, bcastqueue, numbots), dd, locks)
        self.barrier = barrier

    def run(self):

        pid = self.id
        numBots = self.gvh.participants
        lock0 = self.locks[0]
        lock1 = self.locks[1]
        candidate = self.dd.get('candidate')
        if candidate is None:
            candidate = dsm.dsmvar('candidate', -1, 0, ('int', 'aw'), -1)
            self.dd.add(candidate)

        numvoted = self.dd.get('numvoted')
        if numvoted is None:
            numvoted = dsm.dsmvar('numvoted', -1, 0, ('int', 'aw'), -1)
            self.dd.add(numvoted)

        candidate = -1
        numvoted = 0
        voted = False
        leader = None
        while (not (self.stopped())):

            sleep(1)  # adding
            if (self.dd.get("candidate") == None):
                continue

            candidate = self.dd.get("candidate")
            if ((not (voted) and (candidate <= pid))):

                with lock0:

                    candidate = pid
                    self.dd.put("candidate", candidate, time.time())
                    if (self.dd.get("numvoted") == None):
                        continue

                    numvoted = self.dd.get("numvoted")
                    numvoted = (numvoted + 1)
                    self.dd.put("numvoted", numvoted, time.time())
                    voted = True

            # ignoring
            if (self.dd.get("candidate") == None):
                continue

            candidate = self.dd.get("candidate")
            if ((not (voted) and (candidate > pid))):

                with lock1:

                    if (self.dd.get("numvoted") == None):
                        continue

                    numvoted = self.dd.get("numvoted")
                    numvoted = (numvoted + 1)
                    self.dd.put("numvoted", numvoted, time.time())
                    voted = True

            # finalsum
            if (self.dd.get("numvoted") == None):
                continue

            numvoted = self.dd.get("numvoted")
            if ((numvoted == numBots)):

                if (self.dd.get("candidate") == None):
                    continue

                candidate = self.dd.get("candidate")
                leader = candidate









