import sys
import serial
import argparse
from time import sleep

parser = argparse.ArgumentParser(description='Robot Control')
parser.add_argument('--act', type = str, default ="read", help='Work Value')
parser.add_argument('--sen', type = str, default =None, help='Sensor Move, "move"')
parser.add_argument('--wing', type = str, default=None, help='Wing Open/Close')
parser.add_argument('--eye', type=str, default=None, help='eye blink')
parser.add_argument('--led', type = str, default=None , help='led Action')
parser.add_argument('--speak', type = str, default=None, help='Speaker out')

args = parser.parse_args()

sport = serial.Serial("/dev/ttyS0", baudrate=19200, timeout=3.0)

if sport.is_open:
    sport.close()

rdata = bytearray((0xdc, 0x03, 0x00, 0x00, 0x00, 0x04)) # sysid, read, dumy, dumy, dumy, read cnt, not include CRC,  
wdata = bytearray((0xdc, 0x04, 0x00, 0x02, 0x03, 0x04)) # sysid, write, sns_arm, wing, eye, read cnt, not include CRC,  
# sen : 0x01 = sensor move to forward, 0x02 = sensor move to back
# wing : 0x01 = wing Open, 0x02 = wing Close
# eye  : 0x00 = sleep, 0x01 = smail, 0x02 = gloom, 0x03 = wink, 0x04 = blink

if args.sen == "move": 
    wdata[2] = 0x01     #  1
else:
    wdata[2] = 0x00

if args.wing == "open":
    wdata[3] = 0x01
elif args.wing == "close":
    wdata[3] = 0x02
else:
    wdata[3] = 0x00
    
if args.eye == "smail":
    wdata[4] = 0x01
elif args.eye == "gloom":
    wdata[4] = 0x02
elif args.eye == "wink":
    wdata[4] = 0x03
elif args.eye == "blink":
    wdata[4] = 0x04
else:
    wdata[4] = 0x00

if args.act == "write":
    data = wdata
    print("Action write")
else:
    print("Action read")
    data = rdata

print("Data is $s " +  ''.join('{:02x}'.format(x) for x in data))
sport.open()
sport.write(data)
received_data = sport.read()              #read serial port
sleep(0.03)
data_left = sport.inWaiting()             #check for remaining byte
received_data += sport.read(data_left)
print (received_data)    
