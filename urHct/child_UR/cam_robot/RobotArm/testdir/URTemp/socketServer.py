import socket
import URTemp


class SocketServer(object):
    def __init__(self, port = 9876):
        logger = URTemp.logging.getLogger("serverLogger")
        logger.setLevel(logging.INFO)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = ""
        self.port = port
        self.sock.bind((self.host, self.port))

    def waitforconnection(self):
        self.sock.listen(5)
        self.conn, self.addr = self.sock.accept()
        logger.info("Connected by {}".foramt(addr))

        return 1

    def recvmsg(self):
        while True:
            data = self.conn.recv(4096)
            data = data.decode("utf-8")

            if len(data) == 0:
                pass

            return data
            
    
