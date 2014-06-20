import Sensors as sensors
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
edgeSensors = sensors.Reflectance(0)
edgeSensors.start()
while True:
    right, left = edgeSensors.read()
    print 'Right sensor reading: ' + str(right)
    print 'Left sensor reading : ' + str(left)
    time.sleep(1)
