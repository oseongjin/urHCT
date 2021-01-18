import socket, codecs

client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_sock.connect(("localhost", 5011))
camCmd = [0xff, 0xff, 0xdc, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 ]
sndStrmsg = bytes(camCmd).hex()
client_sock.sendall(bytes(sndStrmsg.encode("utf-8")))
data = codecs.decode(client_sock.recv(1024))
tmpRcv = data.split(",")
print("tmpRcv {}".format(tmpRcv))
client_sock.close()
print("finish")
