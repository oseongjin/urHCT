import socket
import threading
import time
from ws2812b import ws2812

host = ""
port = 5000
ws = ws2812()
ws.pixels.fill((255, 0, 0))
ws.pixels.show()

def handle_client(client_socket, addr):
    rcvData = client_socket.recv(1024)
    rcvCmd = rcvData.decode()
    print("Received Data %s \n"%rcvCmd) # ff ff dc 04 00 00 00 00
    if rcvCmd[6:8] == "04":
        print(rcvCmd[14:16])
        tmpcmd = int(rcvCmd[14:16])
        if tmpcmd == 1:
            ws.pixels.fill((255, 0, 0))
            rtnstr = "Red led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            print("Red led on")
        elif tmpcmd == 2:
            ws.pixels.fill((0, 255, 0))
            rtnstr = "green led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            print("green led on")
        elif tmpcmd == 3:
            ws.pixels.fill((0, 0, 255))
            rtnstr = "blue led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            print("blue led on")
        elif tmpcmd == 4:
            ws.rainbow_cycle(0.001)
            rtnstr = "rainbow led on"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            print("rainbow led on")
        else:
            ws.pixels.fill((0, 0, 0))
            rtnstr = "led off"
            client_socket.sendall(bytes(rtnstr.encode("utf-8")))
            print("led off")
    ws.pixels.show()
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

        print("클라이언트 핸들러 스레드로 이동 됩니다.")
        #accept()함수로 입력만 받아주고 이후 알고리즘은 핸들러에게 맡긴다.
        t = threading.Thread(target=handle_client, args=(client_socket, addr))
        t.daemon = True
        t.start()


if __name__ == '__main__':
    accept_func()