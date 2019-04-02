
class Position(list):
    def __init__(self, x, y, z,angle = None):
        self.__x = x
        self.__y = y
        self.__z = z
        self.__angle = angle

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def z(self):
        return self.__z

    @@property
    def angle(self):
        return self.__angle