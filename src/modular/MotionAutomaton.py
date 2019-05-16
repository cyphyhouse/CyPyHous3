import math
import numpy as np
import time
from threading import Thread,Event
from point import point
from state import *


class moat(Thread):
    def __init__(self, pid, rd = 2, configfile = 'testfile',mode = 'Target',route = [],done =False):
        super(moat,self).__init__()
        self.__pid = pid
        self.state = state(pid,configfile)
        self.__rdist = rd
        self.__target = None
        self._stop_event = Event()
        self.__inmotion = False
        self.route = route
        self.mode = mode
        self.done = done

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    @property
    def pid(self):
        """
        getter method for pid
        :return:
        """
        return self.__pid

    @pid.setter
    def pid(self,pid):
        """
        setter method for pid
        :param pid:
        :return:
        """
        self.__pid = pid


    @property
    def rdist(self):
        return self.__rdist

    @rdist.setter
    def rdist(self,rdist):
        self.__rdist = rdist

    @property
    def target(self):
        return self.__target

    @target.setter
    def target(self,target):
        self.__target = target

    @property
    def inmotion(self):
        return self.__inmotion

    @inmotion.setter
    def inmotion(self,inmotion):
        self.__inmotion = inmotion


    def goto(self,pos,Kp_rho,Kp_alpha,Kp_beta,Kp_gamma,dt,ct):

        """
        rho is the distance between the robot and the goal position
        alpha is the angle to the goal relative to the heading of the robot
        beta is the angle between the robot's position and the goal position plus the goal angle
        Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
        Kp_beta*beta rotates the line so that it is parallel to the goal angle
        """
        x_start = self.state.pos.x
        y_start = self.state.pos.y
        z_start = self.state.pos.z
        theta_start = self.state.pos.yaw
        x = x_start
        y = y_start
        z = z_start
        theta = theta_start
        x_goal = pos.x
        y_goal = pos.y
        z_goal = pos.z
        theta_goal = pos.yaw

        x_diff = x_goal - x
        y_diff = y_goal - y
        z_diff = z_goal  - z

        x_traj, y_traj, z_traj , t_traj = [], [], [],[]

        rho = np.sqrt(x_diff ** 2 + y_diff ** 2)
        z_diff = z_goal - z

        while rho > 0.001 :
            x_traj.append(x)
            y_traj.append(y)
            z_traj.append(z)
            t_traj.append(ct)

            x_diff = x_goal - x
            y_diff = y_goal - y
            z_diff = z_goal - z


            vz = z_diff * Kp_gamma
            if abs(z_diff) <= 0.02:
                vz = 0

            # Restrict alpha and beta (angle differences) to the range
            # [-pi, pi] to prevent unstable behavior e.g. difference going
            # from 0 rad to 2*pi rad with slight turn

            rho = np.sqrt(x_diff ** 2 + y_diff ** 2)

            alpha = (np.arctan2(y_diff, x_diff)
                     - theta + np.pi) % (2 * np.pi) - np.pi
            beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

            v = Kp_rho * rho
            w = Kp_alpha * alpha + Kp_beta * beta

            if alpha > np.pi / 2 or alpha < -np.pi / 2:
                v = -v

            theta = theta + w * dt
            x = x + v * np.cos(theta) * dt
            y = y + v * np.sin(theta) * dt
            z = z + vz * dt
            self.state.pos = point.point(x,y,z,theta)
            ct = ct + dt


        return t_traj,x_traj,y_traj,z_traj,len(x_traj)

    def distanceTo(self,point1):
        return distancebetween(point1,self.state.pos)

    def angleto(self,point1):
        return slope(self.state.pos,point1)

    def reached(self,point):
        if point is None:
            return True
        if  distancebetween(self.state.pos,point) <= self.rdist:
            return True
        else:
            return False

    def run(self):
        Kp_rho = 9
        Kp_alpha = 15
        Kp_gamma = 10
        Kp_beta = -3
        dt = 0.01
        ct = 0
        x_traj = []
        y_traj = []
        t_traj = []
        z_traj = []
        path = []
        xvelmax  = 5
        xvelmin = 1.5
        yvelmax = 5
        yvelmin = 1.5

        f = open("positions"+str(self.pid),'w')
        g = open("routes"+str(self.pid),"w")
        while not self.stopped():

            time.sleep(0.001)

            #print(self.pid,self.done)
            if self.mode == 'Target':

                if self.target is not None:
                    if not self.inmotion:
                        t, x, y, z, len1 = self.goto(self.target, Kp_rho, Kp_alpha, Kp_beta, Kp_gamma, dt, ct)
                        t_traj.extend(t)
                        x_traj.extend(x)
                        y_traj.extend(y)
                        z_traj.extend(z)
                        #path.extend([(self.plots[0],self.plots[1]) for i in range(len1)])
                else:
                    #time.sleep(0.001)
                    ct = ct+dt
                    t_traj.append(ct)
                    x_traj.append(self.state.pos.x)
                    y_traj.append(self.state.pos.y)
                    z_traj.append(self.state.pos.z)

                    #path.append((self.plots[0],self.plots[1]))

            elif self.mode == 'Timed':


                self.state.pos.x = self.state.pos.x + self.state.xvel * dt
                self.state.pos.y = self.state.pos.y + self.state.yvel * dt

                self.state.xvel = self.state.xacc * dt + self.state.xvel
                self.state.yvel = self.state.yacc * dt + self.state.yvel
                if self.state.xvel <= xvelmin:
                    if self.state.xacc < 0:
                        self.state.xacc = 0
                    self.state.xvel = xvelmin

                if self.state.xvel >= xvelmax:
                    if self.state.xacc > 0:
                        self.state.xacc = 0
                    self.state.xvel = xvelmax

                if self.state.yvel <= yvelmin:
                    if self.state.yacc < 0:
                        self.state.yacc = 0
                    self.state.yvel = yvelmin

                if self.state.yvel >= yvelmax:
                    if self.xacc > 0:
                        self.xacc = 0
                    self.state.yvel = yvelmax




                #self.state.pos.z = self.state.pos.z + self.state.zvel * dt


                ct = ct + dt
                t_traj.append(ct)
                x_traj.append(self.state.pos.x)
                y_traj.append(self.state.pos.y)
                z_traj.append(self.state.pos.z)
                #path.extend([(self.plots[0], self.plots[1]) for i in range(len1)])



            elif self.mode == 'Routed':
                if not self.route == []:
                    if not self.done:
                        if not self.inmotion:
                            if self.target is None:
                                self.__target = self.route[0]
                                #print("first",self.pid,"going to ",self.target)

                                t, x, y,z, len1 = self.goto(self.target, Kp_rho, Kp_alpha, Kp_beta, Kp_gamma, dt, ct)
                                #print(x,y,z)
                                if len(self.route[1:]) > 0:

                                    self.route = self.route[1:]
                                else:
                                    self.done = True
                                t_traj.extend(t)
                                x_traj.extend(x)
                                y_traj.extend(y)
                                z_traj.extend(z)
                                ct = t_traj[-1]
                                #path.extend([(self.plots[0], self.plots[1]) for i in range(len1)])


                            else:
                                t, x, y,z, len1 = self.goto(self.target, Kp_rho, Kp_alpha, Kp_beta, Kp_gamma, dt, ct)
                                if len(self.route[1:]) > 0:

                                    self.route = self.route[1:]
                                else:
                                    self.done = True
                                t_traj.extend(t)
                                x_traj.extend(x)
                                y_traj.extend(y)
                                z_traj.extend(z)
                                ct = t_traj[-1]
                                #path.extend([(self.plots[0], self.plots[1]) for i in range(len1)])


                    else:
                        self.done = False
                        self.__target = self.route[0]
                        t, x, y, z, len1 = self.goto(self.target, Kp_rho, Kp_alpha, Kp_beta, Kp_gamma, dt, ct)
                        if len(self.route[1:]) > 0:
                            self.route = self.route[1:]
                        else:
                            self.done = True
                        t_traj.extend(t)
                        x_traj.extend(x)
                        y_traj.extend(y)
                        z_traj.extend(z)
                        ct = t_traj[-1]
                        #path.extend([(self.plots[0],self.plots[1]) for i in range(len1)])





                else:
                    #time.sleep(0.001)

                    ct = ct + dt
                    t_traj.append(ct)
                    x_traj.append(self.state.pos.x)
                    y_traj.append(self.state.pos.y)
                    z_traj.append(self.state.pos.z)

                    #path.append((self.plots[0],self.plots[1]))



        #print(self.pid,x_traj,y_traj)
        traj_str = [str(i).replace(")","").replace("(","") for i in zip(t_traj,x_traj,y_traj,z_traj)]
        a = [g.write(str(p[0])+";"+str(p[1])+"\n") for p in path]
        for i in traj_str:
            f.write(i+"\n")
        f.close()
        #f.write(str(t_traj) + "\n" + str(x_traj) + "\n"+ str(y_traj))
        g.close()




