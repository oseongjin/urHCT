import socket
import threading
import time
import cv2
import numpy

TCP_IP = ''
TCP_PORT = 5000

def recvall(count):
    buf = b''
    while count:
        newbuf = server_socket.recvfrom(count)
        #print("newbuf",newbuf)
        if not newbuf: return None
        if type(newbuf) == tuple:
            buf += newbuf[0]
            count -= len(newbuf[0])
        else: 
            buf += newbuf
            count -= len(newbuf)
    return buf

def accept_svr():
    while True:
        try:
            length = recvall(16) 
            #print("length",length)
            if length == 0:
                print("length = 0")
                #if True break the infinite loop
                #print("data empty") 
                break
            
            stringData = recvall(int(length))
            data = numpy.frombuffer(stringData, dtype='uint8')
                
            decimg=cv2.imdecode(data,1)

            cv2.namedWindow('imgServer')
            cv2.moveWindow('imgServer', 0, 0)
            decimg = cv2.resize(decimg, (475, 395)) 
            cv2.imshow('imgServer',decimg)
                
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
        except Exception as e:
            print("resiz")
            #server_socket.close()
            #print("Keyboard interrupt")

if __name__ == '__main__':
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.bind((TCP_IP, TCP_PORT))
        accept_svr()

    except Exception as e:
        print(str(e))
