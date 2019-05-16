from tkinter import *
import socket

print("everything is imported")

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print("socket is established")
#the public ip
host = '127.0.0.1'
port=1234
s.connect((host,port))

print("s.connect done")

def sendShit(event):
    textToSend = T.get("1.0",END)
    s.send(str.encode(textToSend))
    T2.insert(END, s.recv(1024))

print("sendshit defined")

root = Tk()
T = Text(root, height=2, width=30)
T2 = Text(root, height=2, width=30)
B = Button(root, text="Send")
T.pack(side=LEFT,fill=X)
T2.pack(side=TOP,fill=X)
B.pack(side=LEFT,fill=X)
T.insert(END, "Type here")
T2.insert(END, s.recv(1024))
B.bind("<Button-1>",sendShit)
mainloop()
