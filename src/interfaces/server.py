
from tkinter import *
#from mysql.connector import (connection)
import socket
from _thread import *

import sys
root = Tk()
T = Text(root, height=2, width=30)
T2 = Text(root, height=2, width=30)
B = Button(root, text="Send")
T.pack(side=LEFT,fill=X)
T2.pack(side=TOP,fill=X)
B.pack(side=LEFT,fill=X)




#statick ip
host = '127.0.0.1'
port=1234
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.bind((host,port))

try:
    s.bind((host,port))
except socket.error as e:
    print(str(e))

s.listen(5)
print("waiting for connection")
def threaded_client(conn):
    conn.send(str.encode("Connection with the server established\n"))

    while True:
        data = conn.recv(2048)
        reply = "You: " + data.decode('utf-8')
        if not data:
            break
        conn.sendall(str.encode(reply))
    conn.close()

while True:
    conn, addr = s.accept()
    print('connected to: '+ addr[0]+':'+str(addr[1]))
    start_new_thread(threaded_client,(conn,))

root.mainloop()