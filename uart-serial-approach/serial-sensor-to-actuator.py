import serial
import time

def get_output(steps, dir):
    outstring = str(steps) + ',' + ('0' if dir == False else '1') + ';\n'
    return str.encode(outstring)


serSensor = serial.Serial('/dev/ttyACM0')
serActuator = serial.Serial('/dev/ttyACM1')

while True:
    sensorValue = serSensor.readline()
    #time.sleep(15/1000)
    serActuator.write(sensorValue)
    #print("Received " + str(sensorValue))
    while serActuator.inWaiting():
        print("Actuator: " + str(serActuator.readline()))





