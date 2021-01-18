#!/usr/bin/env python
import re, os, sys, subprocess
import time, serial, logging
import socket
import threading
from ws2812b import ws2812

#------------ System logging Setting ----------------------------
rblogger = logging.getLogger("robot")
rblogger.setLevel(logging.INFO)

stream_hander = logging.StreamHandler()
rblogger.addHandler(stream_hander)

#------------SETTINGS AND VARIABLES -------------------------
host = ""
port = 5011

ws = ws2812()
splock = threading.Lock();

sport = serial.Serial("/dev/ttyS0", baudrate=19200, timeout=3.0)

if sport.is_open:
    sport.close()


def playvoice(rcvVar):
    playStr = "/home/pi/work/sound_src/" + str(rcvVar + 300) + ".mp3" 
    voice_proc = subprocess.run(["/usr/bin/mpg123", playStr], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    if voice_proc.returncode == 0:
        rblogger.debug("mp3 proc end!!")
        exit()
    
def rbCtrl(th_socket, addr):
    rblogger.debug("Connected by {}; {}".format(addr[0], addr[1]))
    
    while True:
        try:
            rcvData = th_socket.recv(1024)
                        
            if not rcvData: # rcvData is null or 0, calling socket closed
                rblogger.debug("rcv Thread disconnected by {}; {}".format(addr[0], addr[1]))
                break
            
            tmpList = re.findall('..', rcvData.decode()) # splited by any 2 string(..)
            rcvList = [ int(x, 16) for x in tmpList ]    # convert string element to hex byte
            # [ 0xff, 0xff, 0xdc, 0x04, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00 ]
            #   ext,  ext, sysid, write, arm, wing,  eye, rcnt,  led, voice
            rtnstr = ""
            if rcvList[3] == 4:
                if rcvList[8] == 0:
                    rtnstr = "lnc" # led not change
                elif rcvList[8] == 1:
                    rtnstr = "Rlo" # RED led on
                    ws.pixels.fill((255, 0, 0))
                elif rcvList[8] == 2:
                    rtnstr = "Glo" # Green led on
                    ws.pixels.fill((0, 255, 0))
                elif rcvList[8] == 3:
                    rtnstr = "Blo" # Blue led on
                    ws.pixels.fill((0, 0, 255))
                elif rcvList[8] == 4:
                    rtnstr = "Rbw" # Rainbow led on
                    ws.rainbow_cycle(0.001)
                elif rcvList[8] == 5:
                    rtnstr = "Alo" # All led off
                    ws.pixels.fill((0, 0, 0))
                else:
                    rtnstr = "lnc" # led not change
                    
                ws.pixels.show()
                
                if rcvList[9] != 0:
                    splock.acquire() # Thread lock
                    tvoice = threading.Thread(target=playvoice, args=(rcvList[9], ))
                    tvoice.start()
                    splock.release() # Thread unlock 
                
            sndList = rcvList[2:8]
            if sndList[1] == 3 or sndList[2] != 0 or sndList[3] != 0 or sndList[4] != 0: 
                splock.acquire() # Thread lock
                sport.open()
                sport.write(sndList)
                received_data = sport.read()              #read serial port
                time.sleep(0.03)
                data_left = sport.inWaiting()             #check for remaining byte
                received_data += sport.read(data_left)
                sport.close()
                splock.release() # Thread unlock 
                rtnstr = received_data.decode("utf-8")
            
            th_socket.sendall(bytes(rtnstr.encode("utf-8")))
            
        except ConnectionResetError as e:
            rblogger.debug("Connection and except error {}; {}".format(addr[0], addr[1]))
            break
            
    th_socket.close()

def socket_accept():

    svr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    svr_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    svr_socket.bind((host, port))

    svr_socket.listen(5)

    while True:
        try:
            cli_socket, addr = svr_socket.accept()
        except KeyboardInterrupt:
            svr_socket.close()
            rblogger.error("Keyboard interrupt")

        t = threading.Thread(target=rbCtrl, args=(cli_socket, addr))
        t.daemon = True
        t.start()

if __name__ == '__main__':

    ws.pixels.fill((0, 255, 0))
    ws.pixels.show()
    socket_accept()
