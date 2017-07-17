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
              try:
                newDict = ast.literal_eval(match.group(1))
                self.currentStateDict = newDict
              except:
                pass
            #self.logfile.write(line)

      except (serial.SerialException, serial.serialutil.SerialException,
              serial.SerialTimeoutException):
        print('Serial error in log_serial')
        time.sleep(0.01)
        pass  # Try again

      except UnicodeDecodeError:
        print('Unicode error in log_serial')
        pass  # Try again

      except KeyboardInterrupt:
        interrupted = True
        cleanup()
        raise

  def __init__(self, cfg):
    import tempfile
    from threading import Thread

    device = cfg['serial_device']
    rate = cfg['serial_data_rate']
    self.left_pulse=cfg['steering_actuator_min_pulse']
    self.right_pulse=cfg['steering_actuator_max_pulse']
    self.steering_zero=cfg['steering_actuator_zero_pulse']
    self.steering_range=cfg['steering_actuator_range_limit']
    self.min_pulse=cfg['throttle_actuator_min_pulse']
    self.max_pulse=cfg['throttle_actuator_max_pulse']
    self.throttle_zero=cfg['throttle_actuator_zero_pulse']
    self.throttle_range=cfg['throttle_actuator_range_limit']

    self.inWrite = False
    # Create a logfile
    self.logfile = None
    #self.logfile = tempfile.NamedTemporaryFile(prefix='ser_', dir='.', delete=False)
    self.line_re = re.compile('(\{[^}]+\})')
    self.currentStateDict = {"steering": 0, "throttle": 0}

    # Initialize the serial connection from RPi to its controller
    # Arduino mode is set independently
    self.ser = serial.Serial(device, rate)
    time.sleep(1)
    self.ser.flushInput()

    t = Thread(target=self.log_serial, args=())
    t.daemon = True
    t.start()
    atexit.register(self.cleanup)

  def update(self, throttle=0, angle=0):
    if not self.ser.is_open:
      print("Serial device not open")
      return

    if angle == 0:
      steering_pulse = self.steering_zero
    elif angle < 0:
      steering_pulse = map_range(angle * self.steering_range,
                      0, self.LEFT_ANGLE,
                      self.steering_zero, self.left_pulse)
    else:
      steering_pulse = map_range(angle * self.steering_range,
                      self.RIGHT_ANGLE, 0,
                      self.right_pulse, self.steering_zero)

    if throttle == 0:
      throttle_pulse = self.throttle_zero
    elif throttle > 0:
      throttle_pulse = map_range(throttle * self.throttle_range,
                        0, self.MAX_THROTTLE,
                        self.throttle_zero, self.max_pulse)
    else:
      throttle_pulse = map_range(throttle * self.throttle_range,
                        self.MIN_THROTTLE, 0,
                        self.min_pulse, self.throttle_zero)

    while self.inWrite:
      time.sleep(0.01)
    self.inWrite = True
    #print('\r mapped: angle=%d <- %4.2f, throttle=%d <- %4.2f' % (steering_pulse, angle, throttle_pulse, throttle))

    try:
      msg = "T=%d\n, S=%d\n" % (throttle_pulse, steering_pulse)
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
      diff_st_pulse = 1. * (steering_pulse - self.steering_zero)
      # Traditionally right PWM should be greater than zero and left should be less
      # If not, both the margins will be inverted
      st_left_margin = self.steering_zero - self.left_pulse
      st_right_margin = self.right_pulse - self.steering_zero
      if steering_pulse == 0:
        angle = 0.0
      else:
        # Always map in fraction left to [-1:0] and right to [0:1]
        angle1 = diff_st_pulse/st_left_margin
        angle2 = diff_st_pulse/st_right_margin
        if st_left_margin > 0:
          # Pulses are inverted and calc are -ve, > 0 means left else right
          angle = -angle1 if diff_st_pulse > 0 else -angle2
        else:
          # Pulses are conventional, > 0 means right else left
          angle = angle2 if diff_st_pulse > 0 else angle1

      if throttle_pulse == 0:
        throttle = 0.0
      elif throttle_pulse > self.throttle_zero:
        throttle = 1.*(throttle_pulse - self.throttle_zero)/(self.max_pulse - self.throttle_zero)
      else:
        throttle = 1.*(throttle_pulse - self.throttle_zero)/(self.throttle_zero - self.min_pulse)

    except KeyError:
      print('KeyError')
      angle = 0.0
      throttle = 0.0

    #print('\r mapped: angle=%d -> %4.2f, throttle=%d -> %4.2f' % (steering_pulse, angle, throttle_pulse, throttle))
    return angle, throttle
