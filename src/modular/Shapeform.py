from agentThread import AgentThread
import dsm
import time
from time import sleep
from gvh import gvh
from point import *


class ShapeformApp(AgentThread):

    def __init__(self, name, barrier, numbots=1, sim=True, dd=None, locks=[],index = 0):
        super(ShapeformApp, self).__init__(gvh(name*int(math.sqrt(numbots))+index, 'testfile', numbots, sim,'Target',index), dd, locks)
        #print((name,index),self.gvh.pid)

        self.__barrier = barrier


    def run(self):
       lock0 = self.locks[0]
       n = 0
       pid = self.gvh.pid
       numBots = self.gvh.participants
       a = int(math.sqrt(numBots))

       index = self.gvh.index
       maxid = int(math.sqrt(numBots)) -1
       pos = self.dd.get('pos',pid)
       # print(self.gvh
       id = getid(pid,a,index)
       i = id[0]
       j = id[1]
       nbrs = (getpid((i, j + 1), a),getpid((i, j - 1), a), getpid((i + 1, j), a), getpid((i - 1, j), a))
       if i == 0 and j == 0 :
           nbrs = getpid((maxid,0),a),getpid((0,maxid),a)
       elif i == maxid and j == maxid :
           nbrs = getpid((maxid,0),a),getpid((0,maxid),a)
       elif i == maxid and j == 0:
           nbrs = getpid((maxid,maxid),a),getpid((0,0),a)
       elif j == maxid and i == 0:
           nbrs = getpid((maxid, maxid), a), getpid((0, 0), a)
       elif j == maxid :
           nbrs = (getpid((i-1,j),a), getpid((i+1,j),a))
       elif i == maxid :
           nbrs = (getpid((i,j-1),a), getpid((i,j+1),a))
       elif i == 0:
           nbrs = (getpid((i,j-1),a), getpid((i,j+1),a))
       elif j == 0:
           nbrs = (getpid((i-1,j),a), getpid((i+1,j),a))


       nlist = []
       for nbr in nbrs:
           if nbr >= numBots or nbr < 0:
               pass
           else:
               nlist.append(nbr)
       maxid = int(math.sqrt(numBots)) -1
       pos = self.dd.get('pos',pid)

       if pos is None:
            pos = dsm.dsmvar('pos', pid, self.gvh.moat.state.pos , ('point','ar'), -1)
            self.dd.add(pos)

       while not(self.stopped()):
            sleep(0.01)
            n += 1
            if id == (0,0) or id == (maxid,maxid) or id ==(0,maxid) or id == (maxid,0) :
                #self.gvh.moat.mode = 'Timed'

                self.dd.put('pos',self.gvh.moat.state.pos,time.time(),pid)

            else:
                flag = False
                plist = []
                for nbr in nlist:
                    p = self.dd.get('pos',nbr)
                    if p is None:
                        flag = True
                        break
                    else:
                        plist.append(p)
                        #print(plist)
                if flag:
                    n = n - 1
                    continue
                else:
                    target = midall(plist)
                    self.gvh.moat.target = target

                '''
                else:
                    point1 = self.dd.get('pos',pid+1)
                    point2 = self.dd.get('pos',pid-1)
                    target = midpoint(point1,point2)
                    self.gvh.moat.target = target
                '''
                self.dd.put('pos',self.gvh.moat.state.pos,time.time(),pid)

            print(pid,n)
            if n >= 100:
                self.stop()
                print(pid,"stopping",n)

            try:
                self.__barrier.wait()
            except:
                continue

def midpoint(point1,point2):
    return point((point1.x + point2.x)/2,(point1.y+point2.y)/2)

def midall(plist):
    px = sum([p1.x for p1 in plist])/len(plist)
    py = sum([p1.y for p1 in plist])/len(plist)
    pz = sum([p1.z for p1 in plist])/len(plist)
    return point(px,py,pz)

def getpid(id,numbots):
    return id[0]*numbots + id[1]

def getid(pid,numbots,index):
    a = int((pid - index)/numbots)
    return int((pid - index) / numbots), index
