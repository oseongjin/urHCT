import socket, codecs

client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_sock.connect(("localhost", 5011))
cmdList = [0xff, 0xff, 0xdc, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 ]
cmdList[4] = 2
cmdList[5] = 2
cmdList[6] = 0
cmdList[8] = 2
cmdStr = bytes(cmdList).hex()
client_sock.sendall(bytes(cmdStr.encode("utf-8")))
data = codecs.decode(client_sock.recv(1024))
client_sock.close()
print("finish")
