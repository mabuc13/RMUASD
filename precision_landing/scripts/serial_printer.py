import serial
import time

serIn =  serial.Serial('/dev/tty0',9600)  # open serial port

while True:
    print "Hello"	
    serIn.write(b'hello')
    time.sleep(1)
