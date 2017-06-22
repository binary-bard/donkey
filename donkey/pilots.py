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

class MyKerasCategorical(BasePilot):
    def __init__(self, model_path, **kwargs):
        self.model_path = model_path
        print(self.model_path)
        self.model = None #load() loads the model
        super().__init__(**kwargs)

    def decide(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        # Classes {'rightin': 3, 'rightout': 4, 'middle': 2, 'leftin': 0, 'leftout': 1}
        y = self.model.predict(img_arr)[0]
        angle = round((y[0] - y[3])*1/3 + 2*(y[1] - y[4])/3, 2);
        #print('\r')
        print(round(y[1], 2), round(y[0], 2), round(y[2], 2), round(y[3], 2), round(y[4], 2), angle)
        
        #Fixed throttle for now
        throttle = 0.6
        
        return angle, throttle


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
            pilot = KerasCategorical(d.path, name=d.name, last_modified=last_modified)
            pilot_list.append(pilot)

        print (pilot_list)
        return pilot_list


    def default_pilots(self):
        """ Load pilots from models and add CV pilots """
        pilot_list = self.pilots_from_models()
        #pilot_list.append(OpenCVLineDetector(name='OpenCV'))
        return pilot_list

