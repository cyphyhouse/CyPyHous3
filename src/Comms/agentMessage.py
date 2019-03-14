

class agentMessage():

    def __init__(self,sender,receiver,messageId,contents,timestamp):
        self.__sender = sender
        self.__receiver = receiver
        self.__messageId = messageId
        self.__contents = contents
        self.__timestamp = timestamp

    def __repr__(self):
        s = "msgId: " + str(self.messageId)+ " from: " + str(self.sender) + " to: " + str(self.receiver) + " at: "
        s += str(self.timestamp) + " contents: " + str(self.contents)
        return s

    @property
    def sender(self):
        return self.__sender

    @sender.setter
    def sender(self,sender):
        self.__sender = sender

    @property
    def receiver(self):
        return self.__receiver

    @receiver.setter
    def receiver(self,receiver):
        self.__receiver = receiver

    @property
    def messageId(self):
        return self.__messageId

    @messageId.setter
    def messageId(self,messageId):
        self.__messageId = messageId

    @property
    def contents(self):
        return self.__contents

    @contents.setter
    def contents(self,contents):
        self.__contents = contents

    @property
    def timestamp(self):
        return self.__timestamp

    @timestamp.setter
    def timestamp(self,timestamp):
        self.__timestamp = timestamp


