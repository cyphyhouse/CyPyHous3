import socket
from threading import Thread, Event
import queue


class comms(Thread):

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def wake(self):
        self._sleep_event.set()

    def sleep(self):
        self._sleep_event.clear()

    def is_awake(self):
        return self._sleep_event.is_set()

    def __init__(self,bcastqueue):
        super(comms, self).__init__()
        self.bcastqueue = bcastqueue
        self.daemon = True
        self._stop_event = Event()
        self._sleep_event = Event()
        self.bcastPort = 2562
        self.myLocalIp = socket.gethostbyname(socket.gethostname())

        retries = 0
        err = True
        '''
        self.msocket = None
        while err and retries < 15:
            try:
                self.msocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.msocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                err = False

            except socket.error:
                err = True
                retries += 1
        '''

    def run(self):
        try:
            '''
            self.msocket.bind(('', self.bcastPort))
            '''
            pass
        except:
            print("passing this error")
            pass
        n = 0

        while not (self.stopped()):
            #print("running")
            try:
                #message = "testing "+ str(n)
                if self.is_awake():
                    #self.write(message)
                    self.sleep()
                n += 1
            except socket.error:
                print("error here")


    def setCommsHandler(self):
        pass

    def write(self,message):
        flag = False
        #print(message.contents)
        for a,item in enumerate(self.bcastqueue):
            try:
                if item.contents == message.contents and item.sender == message.sender:
                    item.timestamp = message.timestamp
                    break
                flag = True
            except:
                flag = True
        if not flag:
            self.bcastqueue.append(message)



        '''
        self.msocket.sendto(bytes(str(message), "utf-8"),('192.168.1.255',self.bcastPort))
        '''
