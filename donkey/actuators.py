"""
actuators.py

Classes to control the motors and servos. These classes 
are wrapped in a mixer class before being used in the drive loop.

"""

import time
import sys
import serial, atexit


def map_range(x, X_min, X_max, Y_min, Y_max):
    ''' 
    Linear mapping between two ranges of values 
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x-X_min) / XY_ratio + Y_min) // 1

    return int(y)


    
class RasPiRobot_Controller:
    def __init__(self, driveLeft, driveRight):
        import rrb3
        rr = RRB3(9, 6)
        leftDir = 0
        rightDir = 0
        if driveLeft < 0:  # change direction if number is negative
            leftDir = 1
        if driveRight < 0:
            rightDir = 1
        rr.set_motors(abs(driveLeft), leftDir, abs(driveRight), rightDir)


        
class PCA9685_Controller:
    ''' 
    Adafruit PWM controler. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse) 


        
class PWMSteeringActuator:
    #max angle wheels can turn
    LEFT_ANGLE = -1 
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse


    def update(self, angle):
        #map absolute angle to angle that vehicle can implement.
        pulse = map_range(angle, 
                          self.LEFT_ANGLE, self.RIGHT_ANGLE,
                          self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)



class PWMThrottleActuator:

    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        #super().__init__(channel, frequency)
        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.calibrate()


    def calibrate(self):
        #Calibrate ESC (TODO: THIS DOES NOT WORK YET)
        print('center: %s' % self.zero_pulse)
        self.controller.set_pulse(self.zero_pulse)  #Set Max Throttle
        time.sleep(1)


    def update(self, throttle):
        if throttle > 0:
            pulse = map_range(throttle,
                              0, self.MAX_THROTTLE, 
                              self.zero_pulse, self.max_pulse)
        else:
            pulse = map_range(throttle,
                              self.MIN_THROTTLE, 0, 
                              self.min_pulse, self.zero_pulse)

        sys.stdout.flush()
        self.controller.set_pulse(pulse)
        return '123'



class SerialInterface:
    ''' 
    We need one instance of this class per device to control connection
    '''

    ser = None
    logfile = None

    def log_serial(self):
      '''
      We must read messages from serial device else we may get blocked io
      '''
      interrupted = False
      while not interrupted:
          try:
            #if (self.ser.inWaiting() > 0):
            line=self.ser.readline()
            if len(line):
              if self.log:
                self.logfile.write(line)
              else:
                print(line)
          except (serial.SerialException, serial.SerialTimeoutException):
            pass   #Try again
          except KeyboardInterrupt:
            interrupted = True
            cleanup()
            raise


    def __init__(self, device='/dev/ttyACM0', rate=115200, log=False):
        import tempfile
        from threading import Thread

        self.log = log
        self.firstTime = True
        #Create a logfile
        if log and self.logfile is None:
            self.logfile = tempfile.NamedTemporaryFile(prefix='ser_', dir='.', delete=False)

        # Initialise the serial connection from RPi to its controller
        #print("Initializing serial")
        if self.ser is None:
          try:
            self.ser = serial.Serial(device, rate)
            #Read everything already in pipe and ignore
            while (self.ser.inWaiting() > 0):
              self.ser.readline()
            self.thread = Thread(target = self.log_serial)
            self.thread.start()
            print("Set car in driving mode")

          except:
            print('Unable to open ' + device)
            raise

        atexit.register(self.cleanup)


    def send_msg(self, msg):
      # Do this for only one channel
      if self.firstTime:
        # Ask serial to send debug messages
        self.ser.write('d\n'.encode())
        self.ser.flush()
        self.ser.write('m=1\n'.encode())
        self.ser.flush()
        self.ser.write('n\n'.encode())
        self.ser.flush()
        self.firstTime = False
      self.ser.write(msg.encode())
      self.ser.flush()


    def cleanup(self):
        self.logfile.close()
        self.ser.close()


class Differential_PassThrough_Controller:
    ''' 
    Generic Differential Pass Through Motor Controller 
    For differential drive cars you need one controller for each side.
    PassThrough means the RPi will pass it to its controller via serial interface
    '''
    #This is shared between instances
    serialInterf = None

    def __init__(self, motor_num, device='/dev/ttyACM0', rate=115200):
        # Initialise the serial connection from RPi to its controller
        if Differential_PassThrough_Controller.serialInterf is None:
            Differential_PassThrough_Controller.serialInterf = SerialInterface(device, rate)

        #Motor_num 0 for left, 1 for right
        if motor_num == 0  or motor_num == 1:
          self.motor_num = motor_num
        else:
          raise Exception("invalid motor number")

        self.throttle = 0
        atexit.register(self.set_pulse, pulse=0)
    

    def set_pulse(self, pulse):
        '''
        Required for caliberation
        '''
        self.turn(pulse)
    

    def turn(self, throttle):
        '''
        Update the speed of the motor
        '''
        
        # Direction can be inferred by + or - so use 8bit range
        #throttle = int(map_range(abs(speed), -1, 1, -127, 127))
        if throttle == self.throttle:
          return

        self.throttle = throttle
        if self.motor_num == 0:
          msg = "L="
        else:
          msg = "R="
        msg += str(self.throttle) + '\n'
        #print('\n' + msg)
        Differential_PassThrough_Controller.serialInterf.send_msg(msg)

        
    def test(self, seconds=.5):
        speeds = [-.5, -1, -.5, 0, .5, 1, 0]
        for s in speeds:
            self.turn(s)
            time.sleep(seconds)
            print('speed: %s, throttle=self.throttle' % (s, self.throttle))
        print('motor #%s test complete'% self.motor_num)
        

class PassThrough_Controller:
    ''' 
    Pass control over to motor controller over serial connection. 
    '''
    serialInterf = None

    def __init__(self, channel, device='/dev/ttyACM0', rate=115200):
        # Initialise the serial connection from RPi to its controller
        if PassThrough_Controller.serialInterf is None:
            PassThrough_Controller.serialInterf = SerialInterface(device, rate)

        if channel == 0  or channel == 1:
          self.channel = channel
        else:
          raise Exception("invalid channel")

        self.lastVal = 0

        #channel 0 for throttle and 1 for steering
        atexit.register(self.set_pulse, pulse=0)
    

    def set_pulse(self, pulse):
        #if pulse == self.lastVal:
        #  return
        if self.channel == 0:
          msg = "T="
        else:
          msg = "S="
        msg += str(pulse) + '\n'
        #print('\n' + msg)
        PassThrough_Controller.serialInterf.send_msg(msg)


class Adafruit_Motor_Hat_Controller:
    ''' 
    Adafruit DC Motor Controller 
    For differential drive cars you need one controller for each motor.
    '''
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
        import atexit
        
        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60) 
        
        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num
        
        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0
    

    def turn_off_motors(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)


    def turn(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(map_range(abs(speed), -1, 1, -255, 255))
        
        if speed > 0:            
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)
            
        self.motor.setSpeed(self.throttle)
        
        
    def test(self, seconds=.5):
        speeds = [-.5, -1, -.5, 0, .5, 1, 0]
        for s in speeds:
            self.turn(s)
            time.sleep(seconds)
            print('speed: %s   throttle: %s' % (self.speed, self.throttle))
        print('motor #%s test complete'% self.motor_num)
        

