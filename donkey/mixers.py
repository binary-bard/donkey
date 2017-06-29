'''
mixers.py

Classes to wrap motor controllers into a functional drive unit.
'''

import time
import sys
from donkey import actuators


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
