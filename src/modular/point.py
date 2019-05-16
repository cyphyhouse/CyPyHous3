import math,numpy

class point(list):

    def __init__(self,x,y,z = 0,yaw = numpy.deg2rad(0.0),name='A'):
        """
        point, possibly 3d
        :param x:
        :param y:
        :param name:
        """
        self.__x = x
        self.__y = y
        self.__z = z
        self.__yaw = yaw
        self.__name = name



    @property
    def x(self):
        """
        getter method for x
        :return:
        """
        return self.__x

    @property
    def z(self):
        return self.__z

    @property
    def y(self):
        """
        getter method for y
        :return:
        """
        return self.__y

    @property
    def yaw(self):
        """
        getter method for y
        :return:
        """
        return self.__yaw


    @property
    def name(self):
        """
        getter method for name
        :return:
        """
        return self.__name

    def __repr__(self):
        z_str = ""
        if self.z is not None:
            z_str  = str(self.z)
        return ("position"+ " "+ str(self.name)+" " + str(self.x) + " "+ str(self.y) + z_str)

    @x.setter
    def x(self,x):
        self.__x = x

    @y.setter
    def y(self, y):
        self.__y = y


    @name.setter
    def name(self, name):
        self.__name = name

    def similar(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        else:
            return False

    def __eq__(self,other):
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


def distancebetween(point1,point2):
    a =  abs(math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2))
    #print(point1,point2,a)
    return a



def slope(point1,point2):
    if point1.x == point2.x:
        return 0
    else:
        return math.atan2((point1.y - point2.y),(point1.x - point2.x))




