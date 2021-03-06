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

    #asych img capture from picamera
    mycamera = dk.sensors.PiVideoStream()

    # No need for actuators or controllers for SerialPassthroughMixer
    mymixer = dk.mixers.SerialPassthroughMixer(cfg)

    #setup a local pilot
    mypilot = dk.pilots.KerasPositionalCategorical(model_path=cfg['pilot_model_path'])
    mypilot.load()
    
    #setup the remote host
    myremote = dk.remotes.RemoteClient(remote_url, vehicle_id=cfg['vehicle_id'])

    #Create your car
    car = dk.vehicles.BaseVehicle(drive_loop_delay=cfg['vehicle_loop_delay'],
                                  camera=mycamera,
                                  actuator_mixer=mymixer,
                                  remote=myremote,
                                  pilot=mypilot)
    
    #Start the drive loop
    car.start()
