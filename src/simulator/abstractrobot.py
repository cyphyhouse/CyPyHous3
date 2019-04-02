from abc import ABC,abstractmethod

class abstractrobot(ABC):

    def __init__(self):
        pass


    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def predict(self):
        pass

    @abstractmethod
    def collision(self):
        pass

    @abstractmethod
    def updatePos(self):
        pass

    @abstractmethod
    def inMotion(self):
        pass

    @abstractmethod
    def updateSensor(self,obstaclepositions,sensppointpositions):
        pass