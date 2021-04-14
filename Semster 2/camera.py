'''This script utilizes the camera image, and use CNN to detect image classification'''

from picamera import PiCamera
import time
from picamera.array import PiRGBArray
import cv2

import numpy as np
from tensorflow.keras.preprocessing.image import ImageDataGenerator, img_to_array, load_img
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dropout, Flatten, Dense
from tensorflow.keras import applications

class CNN:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (224, 224)
        self.camera.rotation = 180
    def predict(self):
        
        rawCapture = PiRGBArray(self.camera)
        time.sleep(0.1)

        self.camera.capture(rawCapture, format='bgr')
        image = rawCapture.array
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      
        class_dictionary = np.load('class_indices.npy', allow_pickle=True).item()
        num_classes = len(class_dictionary)
   

        # important! otherwise the predictions will be '0'
  

        image = np.expand_dims(image, axis=0)

        # build the VGG16 network
        model = applications.VGG16(include_top=False, weights='imagenet')

        # get the bottleneck prediction from the pre-trained VGG16 model
        bottleneck_prediction = model.predict(image)

        # build top model
        model = Sequential()
        model.add(Flatten(input_shape=bottleneck_prediction.shape[1:]))
        model.add(Dense(256, activation='relu'))
        model.add(Dropout(0.5))
        model.add(Dense(num_classes, activation='softmax'))

        # use the bottleneck prediction on the top model to get the final
        # classification
        class_predicted = model.predict_classes(bottleneck_prediction)

        probabilities = model.predict_proba(bottleneck_prediction)

        inID = class_predicted[0]

        inv_map = {v: k for k, v in class_dictionary.items()}

        label = inv_map[inID]
        print(label)
        return label

        
if __name__ == '__main__':
    obj = CNN()
    print(obj.predict())


