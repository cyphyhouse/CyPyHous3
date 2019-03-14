import comms
import commHandler
import socket
'''
bcastPort = 2562
myLocalIp = socket.gethostbyname(socket.gethostname())
#print (myLocalIp)
msocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
msocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
msocket.setblocking(0)
try:
    msocket.bind(('', bcastPort))
    while(True):
        print("here")
        message = msocket.recv(bcastPort).decode('utf-8')


except socket.error:
    pass
'''
c = commHandler.commHandler()
c.start()
print("started handler")
c1 = comms.comms()
c1.start()
print("started comms")
