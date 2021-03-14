# -*- coding: utf-8 -*-
"""
used to obtain color of the ball infront
"""

import numpy as np
from keras.preprocessing.image import ImageDataGenerator, img_to_array, load_img
from keras.models import Sequential
from keras.layers import Dropout, Flatten, Dense
from keras import applications
from keras.utils.np_utils import to_categorical
import math
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

camera = PiCamera()
camera.rotation = 180

rawCapture = PiRGBArray(camera)
time.sleep(0.1)