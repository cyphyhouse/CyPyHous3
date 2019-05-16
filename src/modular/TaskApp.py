from agentThread import AgentThread
import dsm
import time,planner
from time import sleep
from gvh import gvh
from point import *
from enum import Enum

class Task(list):
    def __init__(self,loc,asgned,taskId = 0):
        self.loc = loc
        self.asgnId = asgned
        self.taskId = taskId


    def __repr__(self):
        return ("task at "+str(self.loc)+" is assigned to "+str(self.asgnId))

class Stage(Enum):
      ASGN =  1
      GOING = 2
      CMPLT = 3
      ALLDONE = 4
      GOTO = 5


class TaskApp(AgentThread):

    def __init__(self, name, barrier, numbots=1, sim=True, dd=None, locks=[]):
        super(TaskApp, self).__init__(gvh(name, 'testfile', numbots, sim,'Routed'), dd, locks)
        self.__barrier = barrier


    def run(self):
       n = 0
       lock0 = self.locks[0]
       numBots = self.gvh.participants
       pid = self.gvh.pid
       stage = Stage.ASGN
       tasks = self.dd.get('tasks',-1)
       p,px,py= [],[],[]
       obs = [(20,10,4,4), (60,30,5,5) , (110,130,5,5)
       ]
       n = 0
       conflictres = 0.0
       t_real = 0.0

       if tasks is None:
            tasks = dsm.dsmvar('tasks', -1, getTasks() , ('list','aw'), -1)
            self.dd.add(tasks)

       routes = self.dd.get('routes', pid)
       if routes is None:
           routes = dsm.dsmvar('routes', pid, [(self.gvh.moat.state.pos,self.gvh.moat.state.pos)], ('list', 'ar'), -1)
           self.dd.add(routes)

       while not(self.stopped()):
               n = n +1
               t_real = t_real+0.01
               sleep(0.1)
               if stage == Stage.ASGN:
                   tasks = self.dd.get('tasks')
                   alldone = [t.asgnId is not -1 for t in tasks]
                   if all(alldone):
                       stage = Stage.ALLDONE


                   if tasks is None:
                       continue
                   #print(len(tasks))
                   for i in range(len(tasks)):
                       temp = tasks[i]


                       #print(pid,temp.asgnId,temp.loc)
                       print("attempting",n)
                       #print(pid,"attempting",lock0.acquire(False),n,t_real)
                       with lock0:
                           routes = self.dd.getallar('routes',numBots)
                           #print(routes)
                           #(dump p)


                           if temp.asgnId == -1:
                              p = planner.RRT(self.gvh.moat.state.pos, temp.loc, randArea=[-2.0, 4.0],
                                               obstacleList=obs).Planning(animation=False)

                              if notNull(p,routes[:pid]+routes[pid+1:]):
                                  temp.asgnId = pid
                                  self.dd.put('routes',p,time.time(),pid)
                                  tasks[i] = temp
                                  self.dd.put('tasks',tasks,time.time())
                                  stage = Stage.GOTO
                                  break




                       continue

               if stage == Stage.GOTO:
                   #print(pid,"in Goto")
                   self.gvh.moat.route = p
                   self.gvh.moat.done = False
                   stage = Stage.GOING
                   continue

               if stage == Stage.GOING:
                   if self.gvh.moat.done:
                       #print(self.gvh.moat.state.pos,temp.loc)
                       stage = Stage.CMPLT
                       continue
                   else:
                        continue

               if stage == Stage.CMPLT:
                   with lock0:
                       # print(pid,"reached",self.gvh.moat.state.pos)
                       self.dd.put('routes', [(self.gvh.moat.state.pos, self.gvh.moat.state.pos)], time.time(), pid)
                       print(pid,"completed at",n)
                   stage = Stage.ASGN
                   continue

               if stage == Stage.ALLDONE:

                   #print(n,t_real)
                   print(n,pid,"is done",t_real)
                   self.stop()
               #self.barrier



def getTasks():
    tasklines = open('taskfile').readlines()
    tasks = [taskline.strip() for taskline in tasklines]
    tasks = [task.split(';') for task in tasks]
    tasks = [Task(point(float(task[0].split(',')[0]),float(task[0].split(',')[1])),int(task[1])) for task in tasks]
    return tasks

def notNull(path, paths):
    if path is None:
        return False
    for p in paths:
        if planner.pathdistance(p,path,10):
            return False
    return True

