import sys
sys.path.insert(0,'..')

import os
import cv2
import pandas as pd
import shutil
import time
import datetime
from tqdm import tqdm
import matplotlib.pyplot as plt
import glob

from image import *
from LSRecognizer import *

delay = 1

if __name__ == "__main__":
    camera = cv2.VideoCapture(1)
    template = cv2.imread("D:\LSDataset\datatemplate.png")
    counter = 0
    threshold = 0.5
    detected_index = 1
    start_time = time.time()
    while True:
        _,image = camera.read()
        match = ls_match(image,template)
        if get_threshold(match) > threshold:
            counter += 1
            threshold += 0.1
            delay += 1

        else:
            counter = 0
            threshold = 0.5
            delay = 1

        print(get_threshold(match))
        if counter >= 3:
            cv2.imshow('',image)
            cv2.imwrite('detected'+str(detected_index)+'.png')
            print(time.time() - start_time) 
            detected_index += 1
            cv2.waitKey()
            threshold = 0.5
            counter = 0 
            delay = 1
        
        time.sleep(delay)