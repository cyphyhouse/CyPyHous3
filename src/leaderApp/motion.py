from point import *
import model
import time,math
from threading import Thread,Event
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import matplotlib.animation as animation
import motion,point
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt



class motion(Thread):


#0:stopped, 100: x only, 010 : y only , 001 : z only, 110 : xy , 101 : xz , 011 : yz, 111: xyz

    def __init__(self,pos = None,draw = None, route = [],m = None,inmotion= False,done= False,mode=0,going = False,topoint = None):
        super(motion,self).__init__()
        style.use('fivethirtyeight')

        self.fig = plt.figure()
        ax1 = self.fig.add_subplot(1, 1, 1)

        self.model = m
        if self.model is None:
            self.model = model.model(0.2,0.2)
        self.draw = ax1
        self.route = route
        self.pos = pos
        self.inmotion = inmotion
        self._stop_event = Event()
        self.mode = mode
        self.done = done
        self.going = going
        self.topoint = topoint
        self.traj = point.traj()
        self.line1 = []

    def stop(self):
        self._stop_event.set()
        self.__gvh.stop()

    def stopped(self):
        return self._stop_event.is_set()

    def posupdate(self,dt):
        if self.going:
            if self.mode == '0' :
                self.inmotion = False
                self.pos = self.pos

            elif self.mode == '10':
                self.pos = point.point(self.pos.x + self.model.vx * dt,self.pos.y)

            elif self.mode == '01' :
                self.pos = point.point(self.pos.x, self.pos.y + self.model.vy * dt)

            elif self.mode == '11':
                self.pos = point.point(self.pos.x + self.model.vx * dt,self.pos.y + self.model.vy * dt)


            self.traj.append(self.pos)
            #self.traj.append(self.pos)

        return self.line1



    def run(self,dt=1,sleep=0.1):
        while not self.stopped():
            self.posupdate(dt)
            time.sleep(sleep)
            ani = animation.FuncAnimation(self.fig, self.animate, interval=1000)
            plt.show()



    def animate(self):

        xs = self.traj.x_traj
        ys = self.traj.y_traj
        self.draw.clear()
        self.draw.plot(xs, ys)

    def goto(self,point,tolerance):

        dist, distx,disty = distance(self.pos,point)
        self.going = True
        self.topoint = point
        if dist > tolerance:
            self.done= False

        else:
            self.done = True
            self.going = False
            self.mode = '0'

        if distx <= tolerance * math.sqrt(2):
            if disty <= tolerance * math.sqrt(2):
                self.mode = '0'
            else:
                self.mode = '01'
        else:
            if disty <= tolerance * math.sqrt(2):
                self.mode = '10'
            else:
                self.mode = '11'




