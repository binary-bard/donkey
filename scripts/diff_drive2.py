"""
drive.py

Script to run on the Raspberry PI to start your vehicle's drive loop. The drive loop
will use post requests to the server specified in the remote argument. Run the 
serve.py script on a different computer to start the remote server.

Usage:
    drive.py [--remote=<name>] [--config=<name>]

Options:
  --remote=<name>   recording session name
  --config=<name>   vehicle configuration file name (without extension)  [default: vehicle]
"""

import os
from docopt import docopt

import donkey as dk

# Get args.
args = docopt(__doc__)

if __name__ == '__main__':

    #load config file
    cfg = dk.config.parse_config('~/mydonkey/' + args['--config'] + '.ini')

    #get the url for the remote host (for user control)
    remote_url = args['--remote']

    #load the actuators (default is the adafruit servo hat)
    #mythrottlecontroller = dk.actuators.PassThrough_Controller(
    #         cfg['throttle_actuator_channel'], cfg['serial_device'], cfg['serial_data_rate'])
    #mysteeringcontroller = dk.actuators.PassThrough_Controller(
    #         cfg['steering_actuator_channel'], cfg['serial_device'], cfg['serial_data_rate'])
    myleftcontroller = dk.actuators.Differential_PassThrough_Controller(
            cfg['throttle_actuator_channel'], cfg['serial_device'], cfg['serial_data_rate'])
    myrightcontroller = dk.actuators.Differential_PassThrough_Controller(
            cfg['steering_actuator_channel'], cfg['serial_device'], cfg['serial_data_rate'])

    #set the PWM ranges
    #mythrottle = dk.actuators.PWMThrottleActuator(controller=mythrottlecontroller, 
    leftMotor = dk.actuators.PWMThrottleActuator(controller=myleftcontroller, 
                                                  min_pulse=cfg['throttle_actuator_min_pulse'],
                                                  max_pulse=cfg['throttle_actuator_max_pulse'],
                                                  zero_pulse=cfg['throttle_actuator_zero_pulse'])

    #mysteering = dk.actuators.PWMSteeringActuator(controller=mysteeringcontroller,
    rightMotor = dk.actuators.PWMSteeringActuator(controller=myrightcontroller,
                                                  left_pulse=cfg['steering_actuator_min_pulse'],
                                                  right_pulse=cfg['steering_actuator_max_pulse'])

    #abstract class to combine actuators
    #mymixer = dk.mixers.AckermannSteeringMixer(mysteering, mythrottle)
    mymixer = dk.mixers.DifferentialDriveMixer(leftMotor, rightMotor)

    #asych img capture from picamera
    mycamera = dk.sensors.PiVideoStream()
    
    #setup the remote host
    myremote = dk.remotes.RemoteClient(remote_url, vehicle_id=cfg['vehicle_id'])

    #setup a local pilot
    mypilot = dk.pilots.KerasPositionalCategorical(model_path=cfg['pilot_model_path'])
    mypilot.load()

    #Create your car
    car = dk.vehicles.BaseVehicle(drive_loop_delay=cfg['vehicle_loop_delay'],
                                  camera=mycamera,
                                  actuator_mixer=mymixer,
                                  remote=myremote,
                                  pilot=mypilot)
    
    #Start the drive loop
    car.start()
