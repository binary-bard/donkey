'''
vehicles.py

Class to pull together all parts that operate the vehicle including,
sensors, actuators, pilots and remotes.
'''

import time

class BaseVehicle:
    def __init__(self,
                 drive_loop_delay = .05,
                 camera=None,
                 actuator_mixer=None,
                 pilot=None,
                 remote=None):

        self.drive_loop_delay = drive_loop_delay #how long to wait between loops

        #these need tobe updated when vehicle is defined
        self.camera = camera
        self.actuator_mixer = actuator_mixer
        self.pilot = pilot
        self.remote = remote

    def start(self):
        start_time = time.time()
        angle = 0.
        throttle = 0.

        #drive loop
        while True:
            now = time.time()
            start = now

            milliseconds = int( (now - start_time) * 1000)

            #get image array image from camera (threaded)
            img_arr = self.camera.capture_arr()

            # User mode post our RC data to server?
            local_angle, local_throttle = self.actuator_mixer.getValues()

            angle, throttle, drive_mode = self.remote.decide_threaded(img_arr,
                                                 local_angle, 
                                                 local_throttle,
                                                 milliseconds)

            if drive_mode == 'user_local':
                angle, throttle = local_angle, local_throttle

            elif drive_mode == 'local':
                angle, throttle = self.pilot.decide(img_arr)

            elif drive_mode == 'local_angle':
                #only update angle from local pilot
                angle, _ = self.pilot.decide(img_arr)

            if drive_mode != 'user_local':
              self.actuator_mixer.update(throttle, angle)

            #print current car state
            end = time.time()
            lag = end - start
            print('\r CAR: angle: {:+04.2f}   throttle: {:+04.2f}   drive_mode: {}  lag: {:+04.3f}'.format(angle, throttle, drive_mode, lag), end='')
            
            time.sleep(self.drive_loop_delay)
