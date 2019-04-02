import commThread
import socket
import time

class commHandler(commThread.commThread):

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def __init__(self):
        super(commHandler, self).__init__()
        self.bcastPort = 2562
        self.myLocalIp = socket.gethostbyname(socket.gethostname())
        retries = 0
        err = True
        self.msocket = None
        while err and  retries < 15:
            try:
                self.msocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                err = False
            except socket.error:
                err = True
                retries += 1

    def run(self):
        try:
            self.msocket.bind(('', self.bcastPort))
        except:
            pass
        while not (self.stopped()):
            #print("running")
            try:
                message = self.msocket.recvfrom(1024)[0].decode('utf-8')
                time.sleep(1)
                print (message)
            except socket.error:
                pass


    def setCommsHandler(self):
        pass


    def write(self):
        print("here")
        self.msocket.sendto(bytes("testing", "utf-8"),('127.0.0.1',self.bcastPort))
        self.stop()

