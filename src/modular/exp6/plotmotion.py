import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import mpl_toolkits.axes_grid1
import matplotlib.widgets

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import mpl_toolkits.axes_grid1
import matplotlib.widgets

class Player(FuncAnimation):
    def __init__(self, fig, func, frames=None, init_func=None, fargs=None,
                 save_count=None, mini=0, maxi=100, pos=(0.125, 0.92), **kwargs):
        self.i = 0
        self.min=mini
        self.max=maxi
        self.runs = True
        self.forwards = True
        self.fig = fig
        self.func = func
        self.setup(pos)
        FuncAnimation.__init__(self,self.fig, self.update, frames=self.play(), interval =.01,
                                           init_func=init_func, fargs=fargs,
                                           save_count=save_count, **kwargs )

    def play(self):
        while self.runs:
            self.i = self.i+self.forwards-(not self.forwards)
            if self.i > self.min and self.i < self.max:
                yield self.i
            else:
                self.stop()
                yield self.i

    def start(self):
        self.runs=True
        self.event_source.start()

    def stop(self, event=None):
        self.runs = False
        self.event_source.stop()

    def forward(self, event=None):
        self.forwards = True
        self.start()
    def backward(self, event=None):
        self.forwards = False
        self.start()
    def oneforward(self, event=None):
        self.forwards = True
        self.onestep()
    def onebackward(self, event=None):
        self.forwards = False
        self.onestep()

    def onestep(self):
        if self.i > self.min and self.i < self.max:
            self.i = self.i+self.forwards-(not self.forwards)
        elif self.i == self.min and self.forwards:
            self.i+=1
        elif self.i == self.max and not self.forwards:
            self.i-=1
        self.func(self.i)
        self.slider.set_val(self.i)
        self.fig.canvas.draw_idle()

    def setup(self, pos):
        playerax = self.fig.add_axes([pos[0],pos[1], 0.64, 0.04])
        divider = mpl_toolkits.axes_grid1.make_axes_locatable(playerax)
        bax = divider.append_axes("right", size="80%", pad=0.05)
        sax = divider.append_axes("right", size="80%", pad=0.05)
        fax = divider.append_axes("right", size="80%", pad=0.05)
        ofax = divider.append_axes("right", size="100%", pad=0.05)
        sliderax = divider.append_axes("right", size="500%", pad=0.07)
        self.button_oneback = matplotlib.widgets.Button(playerax, label='$\u29CF$')
        self.button_back = matplotlib.widgets.Button(bax, label='$\u25C0$')
        self.button_stop = matplotlib.widgets.Button(sax, label='$\u25A0$')
        self.button_forward = matplotlib.widgets.Button(fax, label='$\u25B6$')
        self.button_oneforward = matplotlib.widgets.Button(ofax, label='$\u29D0$')
        self.button_oneback.on_clicked(self.onebackward)
        self.button_back.on_clicked(self.backward)
        self.button_stop.on_clicked(self.stop)
        self.button_forward.on_clicked(self.forward)
        self.button_oneforward.on_clicked(self.oneforward)
        self.slider = matplotlib.widgets.Slider(sliderax, '',
                                                self.min, self.max, valinit=self.i)
        self.slider.on_changed(self.set_pos)

    def set_pos(self,i):
        self.i = int(self.slider.val)
        self.func(self.i)

    def update(self,i):
        self.slider.set_val(i)


fig = plt.figure()
ax = plt.subplot(111)


def getTasks():
    tasklines = open('taskfile').readlines()
    tasks = [taskline.strip() for taskline in tasklines]
    tasks = [task.split(';') for task in tasks]
    tasksx = [float(task[0].split(',')[0]) for task in tasks]
    tasksy = [float(task[0].split(',')[1]) for task in tasks]

    #tasks = [Task(point(float(task[0].split(',')[0]),float(task[0].split(',')[1])),int(task[1])) for task in tasks]
    #tasks = [Task(point(float(task[0].split(',')[0]),float(task[0].split(',')[1])),task[1]) for task in tasks]
    return list(zip(tasksx,tasksy))

tasks = getTasks()

#tasks = [ax.scatter(t[0],t[1],c='blue') for t in tasks]



start = 0
end = 24

def paddata(l,length):
    if len(l) < length and len(l) >= 1:
        print(length,len(l))
        l.extend([l[len(l) - 1] for i in range(length - len(l))])
    
    return l

tdata = [[float(line.strip().split(',')[0]) for line in open('positions'+str(i)).readlines()] for i in range(start,end+1)]
a = max([len(t) for t in tdata])
xdata = [paddata([float(line.strip().split(',')[1]) for line in open('positions'+str(i)).readlines()],a) for i in range(start,end+1)]
ydata = [paddata([float(line.strip().split(',')[2]) for line in open('positions'+str(i)).readlines()],a) for i in range(start,end+1)]

alldata = list(zip(xdata,ydata))




lines = [ax.plot(data[0],data[1],"grey")[0] for data in alldata]

points = [ax.plot(data[0],data[1],"darkgreen",marker='2',ms=10)[0] for data in alldata]

#obstacles = [ax.plot([20,24],[10,10],"r"),ax.plot([20,20],[10,14],"r"),ax.plot([24,24],[10,14],'r'),ax.plot([20,24],[14,14],'r'),
#             ax.plot([60,65],[30,30],"r"),ax.plot([60,60],[30,35],"r"),ax.plot([65,65],[30,35],'r'),ax.plot([60,65],[35,35],'r'),
#             ax.plot([110,115], [130, 130], "r"), ax.plot([110, 110], [130, 135], "r"), ax.plot([115, 115], [130, 135], 'r'), ax.plot([110, 115],[135,135])]


def init():  # only required for blitting to give a clean slate.
    for line in lines:
    	line.set_data([],[])
    for point in points:
        point.set_data([],[]) 
    return lines+points

def update(i):
    
    for j,line in enumerate(lines):

        if i <= 150:
            line.set_data(alldata[j][0][:i],alldata[j][1][:i])
        else:
            line.set_data(alldata[j][0][i-150:i], alldata[j][1][i-150:i])

    for j,point in enumerate(points):
        try:
            point.set_data(alldata[j][0][i],alldata[j][1][i])
        except:
            pass

ani1 = Player(fig, update, maxi=len(ydata[0])-1)

plt.show()
