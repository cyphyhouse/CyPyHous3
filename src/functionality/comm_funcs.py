# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.


import pickle
import socket

MAX_UDP_SIZE = 65507  # https://en.wikipedia.org/wiki/User_Datagram_Protocol

from src.objects.message import Message


def send(msg: Message, ip: str, port: int, retry=1) -> None:
    """
    :param msg: message to be sent
    :param ip: ip to be sent to
    :param port: port to be sent to
    :param retry: number of re-sends of the message
    :return: Nothing
    TODO: USE different serialization .
    TODO: write test
    """

    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for i in range(retry):
        a = pickle.dumps(msg)
        if len(a) > MAX_UDP_SIZE:
            raise ValueError('Message too large')
        client_sock.sendto(a, (ip, port))
    client_sock.close()
