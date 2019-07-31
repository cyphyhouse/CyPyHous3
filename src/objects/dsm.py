class DSM(object):

    def __init__(self, name, dtype, size, pid, value=None, owner=0, last_updated=0.0):
        # owner 0 for aw, 1 for ar
        self.__name = name
        self.__dtype = dtype
        self.__size = size
        self.__pid = pid
        if owner == 0:
            self.__value = value
            self.__last_updated = last_updated
        else:
            self.__value = []
            self.__last_updated = []
            for i in range(size):
                self.__value.append(None)
                self.__last_updated.append(last_updated)
                if i == pid:
                    self.__value[i] = value

            # print(self.__value)
        self.__owner = owner

    def __repr__(self):
        return (str(self.name) + " " + str(self.value) + " for " + str(self.__pid))

    @property
    def updated(self):
        return self.__last_updated

    @updated.setter
    def updated(self, ts):
        self.__last_updated = ts

    def last_update(self, pid=-1):
        if pid is not -1:
            return self.updated[pid]
        else:
            return self.updated

    def set_update(self, updatets, pid=-1):
        if pid is not -1:
            self.__last_updated[pid] = updatets
        else:
            self.__last_updated = updatets

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def dtype(self):
        return self.__dtype

    @dtype.setter
    def dtype(self, dtype):
        self.__dtype = dtype

    @property
    def value(self):
        return self.__value

    @value.setter
    def value(self, value):
        self.__value = value

    def get_val(self, pid=-1):
        # if type(self.value) is list:
        if self.owner is not 0:
            return self.__value[pid]
        else:
            return self.value

    def set_val(self, value, pid=-1):
        # if type(self.value) is list:
        if self.owner is not 0:
            self.__value[pid] = value
        else:
            self.value = value

    @property
    def owner(self):
        return self.__owner

    @owner.setter
    def owner(self, owner):
        self.__owner = owner
