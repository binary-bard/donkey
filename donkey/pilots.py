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
        super().__init__(**kwargs)

    def decide(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        y = self.model.predict(img_arr)[0]
        #Prediction will have high probability for one class mostly
        idx = np.argmax(y)
        #If camera angle is in center, turn left or right to get the car in center position
        angle = round((2*y[1] + y[0] - y[3] - 2*y[4])/3, 2);
        #If camera angle is in left, we need to turn right to correct angle but if we are already 
        #on right, we need to turn left to fix angle but adjust to position
        angle += round((3*y[6] + 2*y[5] + y[7] - y[9])/3, 2);
        #on left, we need to turn right to fix angle but adjust to position
        angle += round((y[11] - y[12] - 2*y[13] - 3*y[14])/3, 2);
        #print('CAR: class: %s, conf: %.2f angle: %.2f' % (self.dir_map[idx], round(y[idx], 2), angle))
        
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
            pilot = KerasPositionalCategorical(d.path, name=d.name, last_modified=last_modified)
            pilot_list.append(pilot)

        print (pilot_list)
        return pilot_list


    def default_pilots(self):
        """ Load pilots from models and add CV pilots """
        pilot_list = self.pilots_from_models()
        #pilot_list.append(OpenCVLineDetector(name='OpenCV'))
        return pilot_list

