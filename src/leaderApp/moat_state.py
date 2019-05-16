import random,point
MAX_X = 1000
MIN_X = -1000
MAX_Y = 1000
MIN_Y = 1000
XVEL = 2
YVEL = 2
XACC = 1
YACC = 1
TIMETIC = 0.6


class moat_state():
    def __init__(self,pos = point.point(0,0)):
        #print("x and y values set in moat_state")
        self.position = pos
        #print (self.position)
        self.xvel = XVEL
        self.yvel = YVEL
        self.xacc = aXACC
        self.yacc = YACC
        self.timetic = TIMETIC



    def set_position(self,pos):
        self.position = pos

    def set_xvel(self,xvel):
        self.xvel = xvel

    def set_yvel(self,yvel):
        self.yvel = yvel


    def update(self):
        #print("before",self.position)
        self.position = (point.point(self.position.x + self.xvel * self.timetic, self.position.y + self.yvel * self.timetic))
        #print("after",self.position)
        #self.set_xvel(self.xvel + self.xacc * self.timetic)
        #self.set_yvel(self.yvel + self.yacc * self.timetic)

