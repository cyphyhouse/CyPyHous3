import pickle
import socket


def send(msg: str, ip: str, port: int) -> None:
    """
    :param msg: message to be sent
    :param ip: ip to be sent to
    :param port: port to be sent to
    :return: Nothing
    """
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client_sock.sendto(pickle.dumps(msg), (ip, port))

send(msg="hello", ip="<broadcast>", port=2000)
