#!/usr/bin/env python
import socket
import threading
import time
import cv2
import numpy

TCP_IP = ''
TCP_PORT = 5000

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def imagecli(client_socket, addr):
    while True:
        try:
            length = recvall(client_socket,16) 
            if length == 0:
                #if True break the infinite loop
                #print("data empty") 
                break
            
            stringData = recvall(client_socket, int(length))
            data = numpy.frombuffer(stringData, dtype='uint8')
                
            decimg=cv2.imdecode(data,1)

            cv2.namedWindow('imgServer')
            cv2.moveWindow('imgServer', 0, 0)
            decimg = cv2.resize(decimg, (475, 395)) 
            cv2.imshow('imgServer',decimg)
                
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
        except:
            #print("Thread Process stop!!")
            break
            
    client_socket.close()

def accept_svr():
    global server_socket

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((TCP_IP, TCP_PORT))

    server_socket.listen(5)

    while True:
        try:
            imgcli_socket, addr = server_socket.accept()
        except KeyboardInterrupt:
            server_socket.close()
            #print("Keyboard interrupt")

        t = threading.Thread(target=imagecli, args=(imgcli_socket, addr))
        t.daemon = True
        t.start()


if __name__ == '__main__':
    accept_svr()
