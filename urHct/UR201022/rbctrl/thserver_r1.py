import re
import sys
import serial
import socket
import threading
import time
from ws2812b import ws2812

host = ""
port = 5000
ws = ws2812()

sport = serial.Serial("/dev/ttyS0", baudrate=19200, timeout=3.0)

if sport.is_open:
    sport.close()

def handle_client(client_socket, addr):
    rcvData = client_socket.recv(1024)
    tmpList = re.findall('..', rcvData.decode()) # string splited by number 2 (..)
    rcvList = [ int(x, 16) for x in tmpList ]    # convert string element to byte
                                                 # [ 0xff, 0xff, 0xdc, 0x04, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00 ]
    print(rcvList)
    if rcvList[3] == 4:
        if rcvList[8] == 1:
            rtnstr = "Red led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            #print(rtnstr)
            ws.pixels.fill((255, 0, 0))
        elif rcvList[8] == 2:
            rtnstr = "Green led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            #print(rtnstr)
            ws.pixels.fill((0, 255, 0))
        elif rcvList[8] == 3:
            rtnstr = "Blue led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            #print(rtnstr)
            ws.pixels.fill((0, 0, 255))
        elif rcvList[8] == 4:
            rtnstr = "Rainbow led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            #print(rtnstr)
            ws.rainbow_cycle(0.001)
        else:
            rtnstr = "led off"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            #print(rtnstr)
            ws.pixels.fill((0, 0, 0))
        ws.pixels.show()
        
        sndList = rcvList[2:8]
        #print("send cmd {}".format(sndList))
        if sndList[2] != 0 or sndList[3] != 0 or sndList[4] != 0: 
            sport.open()
            sport.write(sndList)
            received_data = sport.read()              #read serial port
            time.sleep(0.03)
            data_left = sport.inWaiting()             #check for remaining byte
            received_data += sport.read(data_left)
            #print (received_data)
            sport.close()
        
    time.sleep(1)
    client_socket.close()

def accept_func():
    global server_socket
    #IPv4 체계, TCP 타입 소켓 객체를 생성
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #포트를 사용 중 일때 에러를 해결하기 위한 구문
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #ip주소와 port번호를 함께 socket에 바인드 한다.
    #포트의 범위는 1-65535 사이의 숫자를 사용할 수 있다.
    server_socket.bind((host, port))

    #서버가 최대 5개의 클라이언트의 접속을 허용한다.
    server_socket.listen(5)

    while True:
        try:
            #클라이언트 함수가 접속하면 새로운 소켓을 반환한다.
            client_socket, addr = server_socket.accept()
        except KeyboardInterrupt:
            server_socket.close()
            print("Keyboard interrupt")

        #print("클라이언트 핸들러 스레드로 이동 됩니다.")
        #accept()함수로 입력만 받아주고 이후 알고리즘은 핸들러에게 맡긴다.
        t = threading.Thread(target=handle_client, args=(client_socket, addr))
        t.daemon = True
        t.start()


if __name__ == '__main__':
    accept_func()