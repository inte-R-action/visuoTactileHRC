#!/usr/bin/env python3
import os
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"   # see issue #152
os.environ["CUDA_VISIBLE_DEVICES"] = ""
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras.models import load_model

dir_path = os.path.dirname(os.path.realpath(__file__))
plt.ion()
#CATEGORIES = ['AllenKeyIn', 'AllenKeyOut', 'ScrewingIn', 'ScrewingOut', 'Null']
#pos = np.arange(len(CATEGORIES))
#model = load_model(f'{dir_path}/channel_scaling_1.h5')
#model.summary()

class imu_classifier:
    def __init__(self, model_file, CATEGORIES, WIN_LEN):
        self.model = load_model(f'{dir_path}/{model_file}')
        self.model.summary()
        self.CATEGORIES = CATEGORIES
        self.pos = np.arange(len(CATEGORIES))
        self.WIN_LEN = WIN_LEN

    def plot_prediction(self, prediction):
        plt.figure(2)
        ax = plt.gca()
        ax.cla()
        ax.bar(self.pos, prediction, align='center', alpha=0.5)
        plt.xticks(self.pos, self.CATEGORIES)
        plt.ylabel('Confidence')
        ax.set_ylim([0, 1])
        plt.pause(0.0001)


    def classify_data(self, new_data, graph):
        new_data = new_data[np.newaxis, ...]
        #print(new_data.shape[0], new_data.shape[1], new_data.shape[2])
        if (new_data.shape[0] == 1) & (new_data.shape[1] == self.WIN_LEN) & (new_data.shape[2] == 18):
            prediction = np.reshape(self.model.predict(new_data), (-1))
            if np.isnan(np.max(prediction)):
                class_predict = 4
            else:
                class_predict = np.argmax(prediction)

            #print(f"Activity Pred: {self.CATEGORIES[class_predict]}, Raw Output: {prediction}")

            if graph:
                self.plot_prediction(prediction)

            return prediction

        else:
            return None
