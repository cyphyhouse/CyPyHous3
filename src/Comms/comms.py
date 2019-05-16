import commThread
import socket


class comms(commThread.commThread):

    def stop(self):
        self.msocket.close()
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def __init__(self,bcastPort):
        super(comms, self).__init__()
        self.bcastPort = bcastPort
        self.myLocalIp = socket.gethostbyname(socket.gethostname())

        retries = 0
        err = True
        self.msocket = None
        while err and retries < 15:
            try:
                self.msocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                err = False

            except socket.error:
                print("socket error")
                err = True
                retries += 1

    def run(self):
        try:
            self.msocket.bind(('', self.bcastPort))
        except:
            print("passing socket binding error on line 37")
            pass
        n = 0

        while not (self.stopped()):
            #print("running")
            try:
                message = "testing "+ str(n)
                self.write(message)
                n += 1
            except socket.error:
                print("error with socket on line 46")


    def setCommsHandler(self):
        pass

    def write(self,message):
        self.msocket.sendto(bytes(message, "utf-8"), ('192.168.1.255',self.bcastPort))

