'''

pilots.py

Methods to create, use, save and load pilots. Pilots 
contain the highlevel logic used to determine the angle
and throttle of a vehicle. Pilots can include one or more 
models to help direct the vehicles motion. 

'''
import os
import math
import random
from operator import itemgetter
from datetime import datetime

import numpy as np
import keras

from donkey import utils

class BasePilot():
    '''
    Base class to define common functions.
    When creating a class, only override the funtions you'd like to replace.
    '''
    def __init__(self, name=None, last_modified=None):
        self.name = name
        self.last_modified = last_modified

    def decide(self, img_arr):
        angle = 0.0
        speed = 0.0
        return angle, speed

    def load(self):
        pass



class KerasCategorical(BasePilot):
    def __init__(self, model_path, **kwargs):
        self.model_path = model_path
        self.model = None #load() loads the model
        super().__init__(**kwargs)


    def decide(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        angle_binned, throttle = self.model.predict(img_arr)
        angle_certainty = max(angle_binned[0])
        angle_unbinned = utils.unbin_Y(angle_binned)
        return angle_unbinned[0], throttle[0][0]


    def load(self):
        self.model = keras.models.load_model(self.model_path)

class KerasPositionalCategorical(BasePilot):
    def __init__(self, model_path, **kwargs):
        self.model_path = model_path
        self.model = None #load() loads the model
        self.dir_map = {0: 'center_leftin', 1: 'center_leftout', 2: 'center_middle', 3: 'center_rightin', 4: 'center_rightout', 5: 'left30_leftin', 6: 'left30_leftout', 7: 'left30_middle', 8: 'left30_rightin', 9: 'left30_rightout', 10: 'right30_leftin', 11: 'right30_leftout', 12: 'right30_middle', 13: 'right30_rightin', 14: 'right30_rightout'}
        self.acc_angle = 0.
        super().__init__(**kwargs)

    def decide(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        y = self.model.predict(img_arr)[0]
        #Prediction will have high probability for one class mostly
        idx = np.argmax(y)
        #print("\rPrediction = ", self.dir_map[idx])

        #If camera angle is in center, turn left or right to get the car in center position
        # Our steering effects are non-linear so use that
        #angle = 0. + 2*y[1] + y[0] - y[3] - 2*y[4];
        #If camera angle is in left, we need to turn right to correct angle 
        #on left position, we need to turn right to adjust position
        #and vice versa
        #angle += 3*y[6] + 2*y[5] + y[7] - y[9];
        #angle += y[11] - y[12] - 2*y[13] - 3*y[14];

        #Positions: lin, lout, mid, rin, rout
        mult = [ .5, .7,  0., -.5, -.7,  # angle = center
                 .8, 1.,  .3,  0., -.4,  # angle = left30
                 .0, .4, -.3, -.8, -1.]  # angle = right30
        # Our left and right are swapped
        angle = np.inner(y, mult)

        #Accumulate angle by number of samples to keep a moving average
        nSamples = 8
        self.acc_angle = ((nSamples - 1.)*self.acc_angle + angle)/nSamples

        # Change throttle according to the angle
        if abs(angle) > 0.6:
          throttle = 0.4
        elif abs(angle) > 0.35:
          throttle = 0.6
        else:
          throttle = 1.0
        
        return round(self.acc_angle, 2), throttle


    def load(self):
        self.model = keras.models.load_model(self.model_path)
        self.model.summary()



class PilotHandler():
    """ 
    Convenience class to load default pilots 
    """
    def __init__(self, models_path):
        self.models_path = os.path.expanduser(models_path)
        
        
    def pilots_from_models(self):
        """ Load pilots from keras models saved in the models directory. """
        models_list = [f for f in os.scandir(self.models_path)]
        pilot_list = []
        for d in models_list:
            last_modified = datetime.fromtimestamp(d.stat().st_mtime)
            pilot = KerasPositionalCategorical(d.path, name=d.name, last_modified=last_modified)
            pilot_list.append(pilot)

        print (pilot_list)
        return pilot_list


    def default_pilots(self):
        """ Load pilots from models and add CV pilots """
        pilot_list = self.pilots_from_models()
        #pilot_list.append(OpenCVLineDetector(name='OpenCV'))
        return pilot_list

