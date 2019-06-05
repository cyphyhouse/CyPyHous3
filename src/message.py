<<<<<<< HEAD
#from typing import NoReturn
=======
>>>>>>> 61583f2bfdb07a9845f090c24b21a6e96deb81ab


class Message(object):
    """
    Message class, we will have parse rules for different message types. A table lookup of parse functions.
<<<<<<< HEAD

    __type: int
    __sender: int
    __content: str
    __timestamp: float
    """
=======
    """
    #__type
    #__sender
    #__content
    #__timestamp
>>>>>>> 61583f2bfdb07a9845f090c24b21a6e96deb81ab

    def __init__(self, m_type: int, sender: int, content: str, ts: float):
        self.__type = m_type
        self.__sender = sender
        self.__content = content
        self.__timestamp = ts

    def __repr__(self):
        return str(self.m_type) + "||" + str(self.sender) + "||" + str(self.content) + "||" + str(self.__timestamp)

    @property
    def m_type(self) -> int:
        """
        getter method for message type
        :return:
        """
        return self.__type

    @m_type.setter
    def m_type(self, m_type: int):  # -> NoReturn:
        """
        setter method for message type
        :param m_type:
        :return:
        """
        self.__type = m_type

    @property
    def sender(self) -> int:
        """
        getter method for sender
        :return:
        """
        return self.__sender

    @sender.setter
    def sender(self, sender: int):  # -> NoReturn:
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
    def content(self, content: str):  # -> NoReturn:
        """
        setter method for content
        :param content:
        :return:
        """
        self.__content = content


def to_msg(msg_str: str) -> Message:
    """
    function to parse a string to message
    :param msg_str: message string representation
    :return:
    """
    msg_items = msg_str.split("||")
    return Message(int(msg_items[0]), int(msg_items[1]), msg_items[2], float(msg_items[3]))
