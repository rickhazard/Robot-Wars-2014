# Robot Wars II - External Sensors
# Author: Rick Phillips
# Date:   2014-05-14
# This collection of classes covers the basic funcionality of some common sensors that can be
# used with the RasPi.
#
# Compass:
# Wraps the hmc58831 magnetometer. It provides the option to set Declination (diff between true north
# and magnetic north) and will return the heading as a tuple (Degrees, Minutes).
# It requires the module i2clibraries, which I did not have in the right location when trying to test
# this.  Hence the entire-class-commented-out thing you see below.
#
# Reflectance (analog reflectance sensor on polulu.com):
# This class performs the heretical operation of applying an analog signal to a digital input.  The GPIO
# pins register a High value at a little over 2 volts, so if you are looking to detect white-->black
# for a low-->high transition then the 3.3v input is sufficient.  Experimentating with 5v seemed to throw
# too many false positives.
# To detect high-->low transition (black-->white), change GPIO.RISING to GPIO.FALLING.
#
# Ultrasonic (ultrasonic sensor available everywhere, I recommend getting one on ebay):
# Simple.  Returns avg distance in cm from sensor to whateve is reflecting the ultrasonic signal.  After
# some experimentation I found it best to do one reading at a time instead of averaging, but your mileage
# may vary.
#
# Enjoy!!
#

import time
import RPi.GPIO as GPIO
## from i2clibraries import i2c_hmc5883l


##class Compass():
##    '''Provide heading info'''
##    def __init__(self, declination = (-13, 27)):
##        '''Default declination for Wilton, CT'''
##        self.hmc5883l = i2c_hmc5883l.i2c_hmc5883l(0)
##        self.hmc5883l.setContinuousMode()
##        self.hmc5883l.setDeclination(declination)
##
##        self.lastHeading  = (0.0, 0.0)      
##        self.curHeading = (0.0, 0.0)
##
##    def getHeading(self):
##        '''Return the current heading as a tuple: (deg, min)'''
##        self.lastHeading = self.curHeading
##        self.curHeading = self.hmc5883l.getHeading()
##        return self.curHeading
##
##    def getDeclination(self):
##        '''Return the declination as a tuple: (deg, min)'''
##        return self.hmc58831.getDeclination()
##
##    def getHeadingDelta(self):
##        '''Get difference between last and current heading'''
##        self.getHeading()
##        degreeDelta = self.curHeading[0] - self.lastHeading[0]
##        minuteDelta = self.curHeading[1] - self.lastHeading[1]
##        return (degreeDelta, minuteDelta)
##
##    def setDeclination(self, declination):
##        '''Set declination for our current location'''
##        self.hmc58831.setDeclination(declination)


        
GPIO_LEFT_EDGE       = 19
GPIO_RIGHT_EDGE      = 21
EDGE_SENSOR_BOUNCE_INTERVAL = 500 # mSec         
     
class Reflectance():
    '''Set up and read reflectance sensor'''
    def __init__(self, value):

        self.debug = True
        self.rightEdge = False
        self.leftEdge = False

        # Not sure if pull down is really necessary here
        GPIO.setup(GPIO_RIGHT_EDGE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(GPIO_LEFT_EDGE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def read(self):
        '''Read and reset edge detection values'''        
        right = self.rightEdge
        left = self.leftEdge
        self.rightEdge = self.leftEdge = False

        if self.debug:
            print 'Reading reflectance sensors'
            print 'Right: ' + str(right)
            print 'Left: ' + str(left)
            
        return right, left            
         
    def start(self):         
        '''Add detection events to watch GPIO pins'''
        GPIO.add_event_detect(GPIO_RIGHT_EDGE,
                              GPIO.RISING,
                              callback=self._setRightEdge,
                              bouncetime=EDGE_SENSOR_BOUNCE_INTERVAL)
        
        GPIO.add_event_detect(GPIO_LEFT_EDGE,
                              GPIO.RISING,
                              callback=self._setLeftEdge,
                              bouncetime=EDGE_SENSOR_BOUNCE_INTERVAL)
        
        if self.debug:
            print 'Started reflectance'

    def exit(self):
        '''Remove detection events from GPIO pins'''
        GPIO.remove_event_detect(GPIO_RIGHT_EDGE)
        GPIO.remove_event_detect(GPIO_LEFT_EDGE)
        if self.debug:
            print 'Exiting reflectance'


    def _setRightEdge(self, value=False):
        '''Set right edge detected border'''
        self.rightEdge = True
        if self.debug:
            print 'Right edge detected line'

    def _setLeftEdge(self, value=False):
        '''Set left edge detected border'''
        self.leftEdge = True
        if self.debug:
            print 'Left edge detected line'
    

GPIO_TRIGGER = 5
GPIO_ECHO    = 3
NUM_SAMPLES  = 3
SPEED_OF_SOUND_CM_SEC = 34300
        
class Ultrasonic():
    '''Set up ultrasonic sensor'''
    def __init__(self):
         
         GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
         GPIO.setup(GPIO_ECHO, GPIO.IN)

         # Set trigger to False and allow time to settle
         GPIO.output(GPIO_TRIGGER, False)
         time.sleep(0.5)

        
    def getDistanceToTarget(self):
        '''Measure distance in cm to nearest target'''

        distanceSum = 0

        for _ in range(0, NUM_SAMPLES):
            
            # Send 10us pulse to trigger
            GPIO.output(GPIO_TRIGGER, True)
            time.sleep(0.00001)
            GPIO.output(GPIO_TRIGGER, False)
            start = time.time()

            while GPIO.input(GPIO_ECHO) == 0:
                start = time.time()

            while GPIO.input(GPIO_ECHO) == 1:
                stop = time.time()

            # calculate pulse length
            elapsed = stop - start

            # Calculate distance to target
            distance = (elapsed * SPEED_OF_SOUND_CM_SEC) / 2
            distanceSum += distance
        # close 'for'

        avgDistance = distanceSum / NUM_SAMPLES

        return avgDistance      

        
        
        
