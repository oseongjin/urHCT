#!/usr/bin/python
import socket
import cv2
import numpy

#socket 수신 버퍼를 읽어서 반환하는 함수
def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

#수신에 사용될 내 ip와 내 port번호
TCP_IP = ''
TCP_PORT = 5001

#TCP소켓 열고 수신 대기
print("Server Start")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
print("listen Start")
s.listen(True)
conn, addr = s.accept()
print("access ip" +  str(addr))
while True:
    #String형의 이미지를 수신받아서 이미지로 변환 하고 화면에 출력
    length = recvall(conn,16) #길이 16의 데이터를 먼저 수신하는 것은 여기에 이미지의 길이를 먼저 받아서 이미지를 받을 때 편리하려고 하는 것이다.
    stringData = recvall(conn, int(length))
    
    if length == 0:
        #if True break the infinite loop
        print("data empty") 
        break
    
    data = numpy.fromstring(stringData, dtype='uint8')
        
    decimg=cv2.imdecode(data,1)
    cv2.imshow('SERVER',decimg)
        
    key = cv2.waitKey(1) & 0xFF
    # check for 'q' key-press
    if key == ord("q"):
        #if 'q' key-pressed break out
        break

s.close()
cv2.destroyAllWindows() 
