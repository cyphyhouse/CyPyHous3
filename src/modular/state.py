import random,point
import config


class state():
    def __init__(self,pid,configfile):
        self.__pos = config.pos(pid,configfile)
        self.__xvel = config.xvel(pid,configfile)
        self.__yvel = config.yvel(pid,configfile)
        self.__xacc = config.xacc(pid,configfile)
        self.__yacc = config.yacc(pid,configfile)
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
    def xvel(self):
        return self.__xvel

    @xvel.setter
    def position(self, xvel):
        self.__xvel = xvel

    @property
    def yvel(self):
        return self.__yvel

    @yvel.setter
    def yvel(self, yvel):
        self.__yvel = yvel

    @property
    def xacc(self):
        return self.__xacc

    @xacc.setter
    def xacc(self, xacc):
        self.__xacc = xacc

    @property
    def yacc(self):
        return self.__yacc

    @yacc.setter
    def position(self, yacc):
        self.__yacc = yacc

    @property
    def timetic(self):
        return self.__timetic

    @timetic.setter
    def timetic(self, timetic):
        self.__timetic = timetic

    def update(self):
        self.__pos = point.point(self.pos.x + self.xvel * self.timetic, self.pos.y + self.yvel * self.timetic)


