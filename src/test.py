import socket,pickle

def send( ip: str, port: int) -> None:
    """
    :param msg: message to be sent
    :param ip: ip to be sent to
    :param port: port to be sent to
    :return:
    """
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client_sock.sendto(pickle.dumps("testing from 0"), (ip, port))


send('192.168.1.255',2000)
