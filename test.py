import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.write(b'NR\n')

while True:
    ser.write(b'NR\n')
    if ser.in_waiting > 0:
        print(ser.readline())