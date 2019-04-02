import agentThread
from time import sleep
from globalVarHolder import gvh


class TestApp(agentThread.agentThread):

    def __init__(self,agentType,agentNum):
        super(TestApp,self).__init__(gvh(str(agentType)+str(agentNum)))
        pass

    def run(self):
       # print("here")
        n = 0
        while not(self.stopped()):
            #print("herenow")
            sleep(0.1)
            n += 1
            print(super().name)
            if n == 6:
                self.stop()









