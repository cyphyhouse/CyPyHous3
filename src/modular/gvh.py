import re
from MotionAutomaton imporrm t *

class gvh(list):

    def __init__(self, name, configfile, participants=1, sim=True, mode = 'Target',index = 0):
        """
        Global Variable Holder keeps system specific information in the robot
        :param name: the robots name
        :param participants: all the other participants
        :param sim: whether this is a simulated gvh
        """
        self.__name = "robot"+str(name)
        self.__sim = sim
        self.__participants = participants
        #pid = int(re.sub(r'[a-zA-Z]', r'', name))
        self.moat = moat(name,4,configfile,mode)
        self.__pid = name
        self.index = index
        self.moat.start()


    @property
    def name(self):
        """
        getter method for name
        :return:
        """
        return self.__name

    @name.setter
    def name(self, name):
        """
        setter method for name
        :param name:
        :return:
        """
        self.__name = name

    @property
    def participants(self):
        """
        getter method for participants
        :return:
        """
        return self.__participants

    @participants.setter
    def participants(self, participants):
        """
        setter method for participants
        :param name:
        :return:
        """
        self.__participants = participants

    @property
    def sim(self):
        """
        getter method for sim
        :return:
        """
        return self.__sim

    @sim.setter
    def sim(self, sim):
        """
        setter method for sim
        :param name:
        :return:
        """
        self.__sim = sim

    @property
    def pid(self):
        """
        getter method for pid
        :return:
        """
        return self.__pid

    @pid.setter
    def pid(self, pid):
        """
        setter method for pid
        :param name:
        :return:
        """
        self.__pid = pid
