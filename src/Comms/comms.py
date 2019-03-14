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
        #self.myLocalIp = socket.gethostbyname(socket.gethostname())
        self.myLocalIp = socket.gethostbyname(socket.gethostname())

        retries = 0
        err = True
        self.msocket = None
        while err and  retries < 15:
            try:
                self.msocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                #self.msocket.setblocking(False)
                # msocket.sendto("test".encode('utf-8'),('255.255.255.255',bcastPort))
                err = False
            except socket.error:
                err = True
                retries += 1

    def run(self):
        self.msocket.bind((self.myLocalIp, self.bcastPort))
        n = 0

        while not (self.stopped()):
            #print("running")
            try:
                message = "testing "+ str(n)
                self.write(message)
                #print("herenow")
                n += 1
            except socket.error:
                print("error here")


    def setCommsHandler(self):
        pass


    def write(self,message):
        #print("here")
        self.msocket.sendto(bytes(message, "utf-8"),('',self.bcastPort))


'''

import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
MESSAGE = "Hello, World!"

print ("UDP target IP:", UDP_IP)
print ("UDP target port:", UDP_PORT)
print ("message:", MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
sock.sendto(MESSAGE.encode('utf-8'), ('<broadcast>', UDP_PORT))
'''