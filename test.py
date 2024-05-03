import serial
import time

ser = serial.Serial("/dev/ttyUSB2", 115200)
time.sleep(1)
ser.write(b'INI\r')