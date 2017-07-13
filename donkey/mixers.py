'''
mixers.py

Classes to wrap motor controllers into a functional drive unit.
'''

import time
import sys
from donkey import actuators
import serial, re, atexit, ast


class BaseMixer():

    def update_angle(self, angle):
        pass

    def update_throttle(self, throttle):
        pass

    def update(self, throttle=0, angle=0):
        '''Convenience function to update
        angle and throttle at the same time'''
        self.update_angle(angle)
        self.update_throttle(throttle)

    def getValues(self):
        angle = 0.0
        throttle = 0.0
        return angle, throttle


class AckermannSteeringMixer(BaseMixer):
    '''
    Mixer for vehicles steered by changing the angle of the front wheels.
    This is used for RC cars
    '''
    def __init__(self, 
                 steering_actuator=None, 
                 throttle_actuator=None):
        self.steering_actuator = steering_actuator
        self.throttle_actuator = throttle_actuator


    def update(self, throttle, angle):
        self.steering_actuator.update(angle)
        self.throttle_actuator.update(throttle)


class DifferentialDriveMixer:
    """
    Mixer for vehicles driving differential drive vehicle.
    Currently designed for cars with 2 wheels.
    """
    def __init__(self, left_motor, right_motor):

        self.left_motor = left_motor
        self.right_motor = right_motor
                
        self.angle=0
        self.left_throttle=0
        self.right_throttle=0
    

    def update(self, throttle, angle):
        self.angle = angle
        
        if throttle == 0 and angle == 0:
           self.stop()
        else:
            #This calculation is dependent on vehicle so we should separate it
            #l_speed = ((self.left_throttle + throttle)/3 + angle/5)
            #r_speed = ((self.right_throttle + throttle)/3 - angle/5)
            if angle < 0.1 and angle > -0.1:
              l_speed = throttle
              r_speed = throttle
            if angle < 0.4 and angle > -0.4:
              l_speed = 0.3 * throttle
              r_speed = 0.8 * throttle
            elif angle < 0.7 and angle > -0.7:
              l_speed = 0
              r_speed = 0.9 * throttle
            else:
              l_speed = 0.9 * throttle
              r_speed = -0.6 * throttle

            #Switch throttles if steering is -ve
            if angle < 0:
              temp = l_speed
              l_speed = r_speed
              r_speed = l_speed

            self.left_throttle = min(max(l_speed, -1), 1)
            self.right_throttle = min(max(r_speed, -1), 1)
            
            self.left_motor.update(self.left_throttle)
            self.right_motor.update(self.right_throttle)
            
            
    def test(self, seconds=1):
        telemetry = [(0, -.5), (0, -.5), (0, 0), (0, .5), (0, .5),  (0, 0), ]
        for t in telemetry:
            self.update(*t)
            print('throttle: %s   angle: %s' % (self.throttle, self.angle))
            print('l_speed: %s  r_speed: %s' % (self.left_throttle, 
                                                self.right_throttle))
            time.sleep(seconds)
            
        print('test complete')
        
        
    def stop(self):
        self.left_motor.update(0)
        self.right_motor.update(0)


def map_range(x, X_min, X_max, Y_min, Y_max):
  '''
  Linear mapping between two ranges of values
  '''
  X_range = X_max - X_min
  Y_range = Y_max - Y_min
  XY_ratio = X_range / Y_range

  y = ((x - X_min) / XY_ratio + Y_min) // 1

  return int(y)


class SerialPassthroughMixer(BaseMixer):
  '''
  Throttle and steering are controlled by Arduino. This just passes values over or reads back
  '''

  LEFT_ANGLE = -1
  RIGHT_ANGLE = 1
  MIN_THROTTLE = -1
  MAX_THROTTLE = 1

  def log_serial(self):
    '''
    We must read messages from serial device else we may get blocked io
    '''
    interrupted = False
    while not interrupted:
      try:
        if self.ser is None or self.ser.in_waiting <= 0:
          time.sleep(0.005)
        else:
          line = self.ser.readline().decode()
          if len(line):
            match = self.line_re.search(line)
            if match is not None:
              self.currentStateDict = ast.literal_eval(match.group(1))
            #self.logfile.write(line)

      except (serial.SerialException, serial.serialutil.SerialException,
              serial.SerialTimeoutException):
        time.sleep(0.01)
        pass  # Try again

      except KeyboardInterrupt:
        interrupted = True
        cleanup()
        raise

  def __init__(self, device='/dev/serial0', rate=115200, left_pulse=290, right_pulse=490,
               max_pulse=300, min_pulse=490, zero_pulse=350):
    import tempfile
    from threading import Thread

    self.inWrite = False
    # Create a logfile
    self.logfile = None
    #self.logfile = tempfile.NamedTemporaryFile(prefix='ser_', dir='.', delete=False)
    self.line_re = re.compile('(\{[^}]+\})')
    self.currentStateDict = {}

    self.left_pulse = left_pulse
    self.right_pulse = right_pulse
    self.max_pulse = max_pulse
    self.min_pulse = min_pulse
    self.zero_pulse = zero_pulse

    # Initialise the serial connection from RPi to its controller
    # Arduino mode is set independently
    self.ser = serial.Serial(device, rate)
    time.sleep(1)
    self.ser.flushInput()
    t = Thread(target=self.log_serial, args=())
    t.daemon = True
    t.start()
    atexit.register(self.cleanup)

  def update(self, throttle, angle):
    if not self.ser.is_open:
      print("Serial device not open")
      return

    steering_pulse = map_range(angle,
                      self.LEFT_ANGLE, self.RIGHT_ANGLE,
                      self.left_pulse, self.right_pulse)
    if throttle > 0:
      throttle_pulse = map_range(throttle,
                        0, self.MAX_THROTTLE,
                        self.zero_pulse, self.max_pulse)
    else:
      throttle_pulse = map_range(throttle,
                        self.MIN_THROTTLE, 0,
                        self.min_pulse, self.zero_pulse)

    while self.inWrite:
      time.sleep(0.001)
    self.inWrite = True
    try:
      msg = "T=%d\n" % throttle
      self.ser.write(msg.encode())
      self.ser.flush()
      msg = "S=%d\n" % angle
      self.ser.write(msg.encode())
      self.ser.flush()

    except serial.SerialException:
      print("Failed to write to Serial device")

    self.inWrite = False

  def cleanup(self):
    if self.logfile is not None:
      self.logfile.close()
    self.ser.close()

  def getValues(self):
    try:
      steering_pulse = self.currentStateDict['steering']
      throttle_pulse = self.currentStateDict['throttle']
      if steering_pulse == 0:
        angle = 0.0
      else:
        angle = (2.*steering_pulse - (self.left_pulse + self.right_pulse))/(self.left_pulse - self.right_pulse)

      if throttle_pulse == 0:
        throttle = 0.0
      elif throttle_pulse > self.zero_pulse:
        throttle = 1.*(throttle_pulse - self.zero_pulse)/(self.max_pulse- self.zero_pulse)
      else:
        throttle = 1.*(throttle_pulse - self.zero_pulse)/(self.min_pulse - self.zero_pulse)

    except KeyError:
      print('KeyError')
      angle = 0.0
      throttle = 0.0

    #print('angle=%d, throttle=%d' % (angle, throttle))
    return angle, throttle
