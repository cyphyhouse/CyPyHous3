from commHandler import CommHandler
from message import  Message
import time

a = CommHandler('127.0.0.1',2003,2004,0)
b = CommHandler('127.0.0.1',2004,2003,1)


a.send(Message(0,0,"hello",time.time()))
