import agentThread
from time import sleep


class TestApp(agentThread.agentThread):

    def testApp(self):
        pass

    def run(self):
       # print("here")
        n = 0
        while(not(self.stopped())):
            print("herenow")
            sleep(0.1)
            n += 1
            print(super().name)
            if n == 6:
                self.stop()









