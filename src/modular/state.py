import random,point
import config


class state():
    def __init__(self,pid,configfile):
        self.__pos = config.pos(pid,configfile)
        self.xvel = config.xvel(pid,configfile)
        self.yvel = config.yvel(pid,configfile)
        self.xacc = config.xacc(pid,configfile)
        self.yacc = config.yacc(pid,configfile)
        self.__timetic = config.timetic(pid,configfile)
        self.__speed = config.speed(pid,configfile)

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self,speed):
        self.__speed = speed

    @property
    def pos(self):
        return self.__pos

    @pos.setter
    def pos(self,pos):
        self.__pos = pos

    @property
    def timetic(self):
        return self.__timetic

    @timetic.setter
    def timetic(self, timetic):
        self.__timetic = timetic

    def update(self):
        self.xvel = self.xvel + self.xacc * self.timetic
        self.yvel = self.yvel + self.yacc * self.timetic
        self.__pos = point.point(self.pos.x + self.xvel * self.timetic, self.pos.y + self.yvel * self.timetic)


