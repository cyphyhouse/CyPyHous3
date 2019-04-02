import math
class point(list):

    def __init__(self,x,y,name = 'A'):
        self.__x = x
        self.__y = y
        self.__name = name



    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y


    def __repr__(self):
        return ("position"+ " "+ str(self.name)+" " + str(self.x) + " "+ str(self.y))


    @property
    def name(self):
        return self.__name

    @x.setter
    def x(self,x):
        self.__x = x

    @y.setter
    def y(self, y):
        self.__y = y

    @name.setter
    def name(self, name):
        self.__name = name

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        else:
            return False

    def same(self,other):
        if self.x == other.x and self.y == other.y and self.name == other.name:
            return True
        else:
            return False


class traj():
    def __init__(self):
        self.__xtraj = []
        self.__ytraj = []

    def append(self,pos):
        self.__xtraj.append(pos.x)
        self.__ytraj.append(pos.y)


    @property
    def x_traj(self):
        return self.__xtraj

    @property
    def y_traj(self):
        return self.__ytraj


def distance(point1,point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2),point2.x - point1.x, point2.y - point1.y

