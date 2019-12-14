# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from src.datatypes.message_types import MsgType


class Message(object):
    """
    message format for CyPhyHouse messages

    __sender : sender of the message
    __message_type : message type
    __content : message contents
    __timestamp : message timestamp

    TODO: move to abstract class, make all messages strings, stop pickling, use msgpack to serialize messages.
    """

    def __init__(self, sender: int, message_type: MsgType, content, timestamp: float):
        """
        :param sender: integer message sender agent pid
        :type sender: int

        :param message_type: message type
        :type message_type: MsgType

        :param content:  message content
        :type content: Any

        :param timestamp: float timestamp
        :type timestamp: float
        """
        self.__sender = sender
        self.__message_type = message_type
        self.__content = content
        self.__timestamp = timestamp

    def __repr__(self):
        """
        string representation
        """
        return str(self.sender) + " , " + str(self.message_type) + " , " + str(self.content)

    def __eq__(self, other):
        """
        equality
        """
        try:
            return self.sender == other.sender and \
                   self.message_type == other.message_type and \
                   self.content == other.content and \
                   self.timestamp == other.timestamp
        except AttributeError:
            return False

    # ------------ MEMBER ACCESS METHODS --------------

    @property
    def sender(self) -> int:
        return self.__sender

    @property
    def timestamp(self):
        return self.__timestamp

    @property
    def content(self):
        return self.__content

    @property
    def message_type(self) -> MsgType:
        return self.__message_type

    @sender.setter
    def sender(self, sender: int) -> None:
        self.__sender = sender

    @content.setter
    def content(self, content) -> None:
        self.__content = content

    @message_type.setter
    def message_type(self, message_type: MsgType) -> None:
        self.__message_type = message_type
