#!/usr/bin/python
import socket
import cv2
import numpy

#연결할 서버(수신단)의 ip주소와 port번호
TCP_IP = '172.30.1.56'
TCP_PORT = 5001
capture = cv2.VideoCapture(0)

#송신을 위한 socket 준비
sock = socket.socket()
sock.connect((TCP_IP, TCP_PORT))

while True:
    #OpenCV를 이용해서 webcam으로 부터 이미지 추출
    ret, frame = capture.read()

    #추출한 이미지를 jpg로 압축하고 String 형태로 변환(인코딩)시키는 과정
    encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),90]
    result, imgencode = cv2.imencode('.jpg', frame, encode_param)
    #압축된 이미지를 numpy 형태의 array로 변환후 String으로 변환
    data = numpy.array(imgencode)
    stringData = data.tostring()

    #전송의 안정성을 위해 먼저 전송할 데이터의 크기를 전송 
    tmpstr = str(len(stringData)).ljust(16)
    
    try:
        sock.send( tmpstr.encode('utf-8'));
        #수신준비된 서버에 스트링 포맷의 프래임 데이터 전송
        sock.send( stringData );
    except:
        print("Connection has been disconnected")
        sock.close()
        break

