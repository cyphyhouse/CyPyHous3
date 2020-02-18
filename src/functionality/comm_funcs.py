# Copyright (c) 2019 CyPhyHouse. All Rights Reserved.


import pickle
import socket

import src.objects.message as message

MAX_UDP_SIZE = 65507  # https://en.wikipedia.org/wiki/User_Datagram_Protocol


def send(msg: message.Message, ip: str, port: int, retry=1) -> None:
    """
    :param msg: message to be sent
    :type msg: Message

    :param ip: ip to be sent to
    :type ip: str

    :param port: port to be sent to
    :type port: int

    :param retry: number of re-sends of the message
    :type retry: int

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