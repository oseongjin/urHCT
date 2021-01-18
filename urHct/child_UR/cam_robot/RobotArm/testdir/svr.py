import URTemp

svrsock = URTemp.socketServer.SocketServer(6001)
svrsock.waitforconnection()
while True:
    print("while start")
    data = recvmsg()
    print("data")
