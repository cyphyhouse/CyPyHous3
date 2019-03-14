
import commThread
import socket


class comms(commThread.commThread):

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def __init__(self):
        super(comms, self).__init__()
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
                self.write()
                # msocket.sendto("test".encode('utf-8'),('255.255.255.255',bcastPort))
                err = False
            except socket.error:
                err = True
                retries += 1

    def run(self):
        while not (self.stopped()):
            print("running")
            try:
                self.msocket.bind(('',self.bcastPort)

                message = self.msocket.recvfrom(self.bcastPort).decode('utf-8')
                print (message)
                print("herenow")
            except socket.error:
                pass


    def setCommsHandler(self):
        pass


    def write(self):
        self.msocket.sendto("test".encode('utf-8'),('255.255.255.255',self.bcastPort))


