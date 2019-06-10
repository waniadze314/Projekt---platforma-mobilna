import sys
sys.path.insert(0,'..')

import os
import cv2
import shutil
import time
import datetime
import matplotlib.pyplot as plt
import glob

from image import *
from LSRecognizer import *

def recognize(camera,template):
    delay = 1 # delay time in seconds
    counter = 0 # counter collecting detected images
    threshold = 0.5 # initial threshold for gradient recognizer
    detected_index = 1 # utility index for saving detected images to directory
    # greater value gives more certanity for detecting image 
    # but lower probability forlow quality images
    detection_constant = 3 
    start_time = time.time() # start time 
    # Main loop
    while time.time() - start_time <= 20.0:
        _,image = camera.read() # take picture
        match = ls_match(image,template) # match with gradient recognizer
        # if image is matched update counter
        if get_threshold(match) > threshold: 
            counter += 1
            threshold += 0.1 # increase threshold to avid false detections
            delay += 1 # icrease delay to allow robot for better positiong
        # if initial detection was false
        else:
            counter = 0 # reset counter threshold and delay
            threshold = 0.5
            delay = 1

        print(get_threshold(match))
        
        # if image was detected
        if counter >= detection_constant:
            # display image
            # cv2.imshow('',image)
            # # cv2.waitKey()
            # cv2.imwrite('detected'+str(detected_index)+'.png')
            # # save image to file
            # with open("times.txt","a") as times_file:
            #     times_file.write(str(time.time() - start_time))
            
            return True
        # wait 
        time.sleep(delay)

    return False

if __name__ == "__main__":
    file_delay = 5.0
    camera = cv2.VideoCapture(0)
    templateG = cv2.imread("C:\\Users\\Krzysiek\\Documents\\GitHub\\TeamProject\\datatemplate.png")
    signal = True

    if signal == True:
        out = recognize(camera,templateG)
        if out == True:
            with open("LabelG.txt","w") as f:
                f.write("True")

    time.sleep(file_delay)
    with open("LabelG.txt","w") as f:
        f.write("False")