class Message(object):
    """
    message format for cyphyhouse

    __sender : sender of the message
    __message_type : message type
    __content : message contents
    __timestamp : message timestamp
    TODO: How to type hint content.
    """

    def __init__(self, sender: int, message_type: int, content, timestamp: float):
        """
        init method for message
        :param sender: integer message sender agent pid
        :param message_type: integer message type
        :param content:  message content
        :param timestamp: float timestamp
        """
        self.__sender = sender
        self.__message_type = message_type
        self.__content = content
        self.__timestamp = timestamp

    def __repr__(self):
        """
        string representation
        :return:
        """
        return str(self.sender) + " , " + str(self.message_type) + " , " + str(self.content)

    @property
    def sender(self) -> int:
        """
        getter method for message sender
        :return: integer message sender
        """
        return self.__sender

    @sender.setter
    def sender(self, sender: int) -> None:
        """
        setter method for message sender
        :param sender: integer sender
        :return: None
        """
        self.__sender = sender

    @property
    def content(self):
        """
        getter method for message content
        :return: message contents
        """
        return self.__content

    @content.setter
    def content(self, content) -> None:
        """
        setter method for message content
        :param content: content
        :return: None
        """
        self.__content = content

    @property
    def message_type(self) -> int:
        """
        getter method for message type
        :return: integer message type
        """
        return self.__message_type

    @message_type.setter
    def message_type(self, message_type: int) -> None:
        """
        setter method for message type
        :param message_type: integer message type
        :return: None
        """
        self.__message_type = message_type

    @property
    def timestamp(self):
        """
        getter method for timestamp
        :return:
        """
        return self.__timestamp
