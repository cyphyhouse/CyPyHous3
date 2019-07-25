import pickle
import socket


def receive():
    """
    create receiver socket, receive messages.
    :return:
    """
    receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    try:
        receiver_socket.bind(("", 2000))
        receiver_socket.settimeout(10.0)
    except OSError:
        print("perhaps already created socket")

    try:
        data, addr = receiver_socket.recvfrom(4096)
        msg = pickle.loads(data)
        print(msg)
        # self.agent_gvh.add_recv_msg(msg)
    except socket.timeout:
        print("error")
        # print("agent", self.agent_gvh.pid, "timed out")
    except OSError:
        print("error")
        # print("unexpected os error on agent", self.agent_gvh.pid)

    receiver_socket.close()


receive()
