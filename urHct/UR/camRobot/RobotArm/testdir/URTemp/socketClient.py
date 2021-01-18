import socket
import URTemp

class SocketClient(object):
    def __init__(self, host = "localhost", port = "1234"):
      logger = URTemp.logging.getLogger("clientLogger")
      logger.setLevel(logging.INFO)
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.host = host
      self.port = port
      
    def connectSock(self):
        self.sock.connect((self.host, self.port))

    def sendData(self):
        data = bytes(data,encoding='utf-8')
        client_socket.sendall(data)

    def closeSock():
        self.sock.close()


