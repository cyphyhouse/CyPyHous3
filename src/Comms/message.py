from typing import NoReturn


class Message(object):
    """
    Message class, we will have parse rules for different message types. A table lookup of parse functions.
    """
    __type: int
    __sender: int
    __content: str
    __timestamp: float

    def __init__(self, mtype: int, sender: int, content: str, ts: float):
        self.__type = mtype
        self.__sender = sender
        self.__content = content
        self.__timestamp = ts

    def __repr__(self):
        return str(self.mtype) + "||" + str(self.sender) + "||" + str(self.content) + "||" + str(self.__timestamp)

    @property
    def mtype(self) -> int:
        """
        getter method for message type
        :return:
        """
        return self.__type

    @mtype.setter
    def mtype(self, mtype: int) -> NoReturn:
        """
        setter method for message type
        :param mtype:
        :return:
        """
        self.__type = mtype

    @property
    def sender(self) -> int:
        """
        getter method for sender
        :return:
        """
        return self.__sender

    @sender.setter
    def sender(self, sender: int) -> NoReturn:
        """
        setter method for sender
        :param sender:
        :return:
        """
        self.__sender = sender

    @property
    def content(self) -> str:
        """
        getter method for message contents
        :return:
        """
        return self.__content

    @content.setter
    def content(self, content: str) -> NoReturn:
        """
        setter method for content
        :param content:
        :return:
        """
        self.__content = content


def toMsg(msgstr: str) -> Message:
    """
    function to parse a string to message
    :param msgstr: message string representation
    :return:
    """
    msgitems = msgstr.split("||")
    return Message(int(msgitems[0]), int(msgitems[1]), msgitems[2], float(msgitems[3]))
