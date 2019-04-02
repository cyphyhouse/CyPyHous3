import random,point
MAX_X = 1000
MIN_X = -1000
MAX_Y = 1000
MIN_Y = 1000
XVEL = 0
YVEL = 0
XACC = 0
YACC = 0
TIMETIC = 1

class moat_state():
    def __init__(self,x = 0,y = 0):
        #print("x and y values set in moat_state")
        self.__position = point.point(x,y)
        print (self.__position)
        self.__xvel = XVEL
        self.__yvel = YVEL
        self.__xacc = XACC
        self.__yacc = YACC
        self.__timetic = TIMETIC

    @property
    def position(self):
        return self.__position

    @property
    def xvel(self):
        return self.__xvel

    @property
    def yvel(self):
        return self.__yvel

    @property
    def xacc(self):
        return self.__xacc

    @property
    def yacc(self):
        return self.__yacc

    @property
    def timetic(self):
        return self.__timetic

    @position.setter
    def position(self,pos):
        self.__position = pos

    @xvel.setter
    def xvel(self,xvel):
        self.__xvel = xvel

    @yvel.setter
    def yvel(self,yvel):
        self.__yvel = yvel

    @xacc.setter
    def xacc(self,xacc):
        self.__xacc = xacc

    @yacc.setter
    def yacc(self,yacc):
        self.__yacc = yacc

    @timetic.setter
    def timetic(self,timetic):
        self.__timetic = timetic

    def update(self):
        xvelnew = self.xvel(xvel + self.xacc * self.timetic)
        yvelnew = self.yvel(yvel + self.yacc * self.timetic)
        self.position((self.position[0] + self.xvel * self.timetic, self.position[1] + self.yvel * self.timetic))
        self.xvel(xvelnew)
        self.yvel(yvelnew)
