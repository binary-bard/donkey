"""
Script to run on the Raspberry PI to start your vehicle's drive loop. The drive loop
will use post requests to the server specified in the remote argument. Use the
serve.py script to start the remote server.

Usage:
    diff_drive.py [--remote=<name>] [--config=<name>]


Options:
  --remote=<name>   recording session name
  --config=<name>   vehicle configuration file name (without extension)  [default: diff_vehicle]
"""

import os
from docopt import docopt

import donkey as dk

# Get args.
args = docopt(__doc__)

if __name__ == '__main__':

    cfg = dk.config.parse_config('~/mydonkey/' + args['--config'] + '.ini')

    remote_url = args['--remote']

    left_motor = dk.actuators.Differential_PassThrough_Controller(
            cfg['throttle_actuator_channel'], cfg['serial_device'], cfg['serial_data_rate'])
    right_motor = dk.actuators.Differential_PassThrough_Controller(
            cfg['steering_actuator_channel'], cfg['serial_device'], cfg['serial_data_rate'])

    dd = dk.mixers.DifferentialDriveMixer(left_motor=left_motor,
                                 right_motor =right_motor)

    #asych img capture from picamera
    mycamera = dk.sensors.PiVideoStream()

    #Get all autopilot signals from remote host
    myremote = dk.remotes.RemoteClient(remote_url, vehicle_id=cfg['vehicle_id'])

    #setup a local pilot
    mypilot = dk.pilots.KerasCategorical(model_path=cfg['pilot_model_path'])
    mypilot.load()

    #Create your car
    car = dk.vehicles.BaseVehicle(drive_loop_delay=cfg['vehicle_loop_delay'],
                                  camera=mycamera,
                                  actuator_mixer=dd,
                                  remote=myremote,
                                  pilot=mypilot)
    car.start()
