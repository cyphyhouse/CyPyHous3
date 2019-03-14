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
        self.myLocalIp = '127.0.0.1' #socket.gethostbyname(socket.gethostname())
        retries = 0
        err = True
        self.msocket = None
        while err and  retries < 15:
            try:
                self.msocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                #self.msocket.setblocking(False)
                #self.write()
                #message = self.msocket.recvfrom(1024).decode('utf-8')
                #print("here")
                # msocket.sendto("test".encode('utf-8'),('255.255.255.255',bcastPort))
                err = False
            except socket.error:
                err = True
                retries += 1

    def run(self):
        self.msocket.bind((self.myLocalIp, self.bcastPort))
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

'''
import socket

UDP_IP = socket.gethostbyname(socket.gethostname())
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print ("received message:", data.decode('utf-8'))

'''
