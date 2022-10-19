import serial
import time

ser = serial.Serial('/dev/ttyACM0')
ser.write(str.encode("Start!\n"))
time.sleep(0.5)

while True:
    input = ser.readline()
    if input[0] == 0x54:
        ser.write(input)
    else:
        print("Teensy: " + str(input))
