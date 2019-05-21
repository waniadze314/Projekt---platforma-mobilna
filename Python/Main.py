import sys
sys.path.insert(0,'..')

import mxnet as mx
from mxnet import nd
from mxnet import autograd
from mxnet import gluon
from mxnet import init

from mxnet.gluon import data as gluon_data
from mxnet.gluon import loss as gluon_loss
from mxnet.gluon import nn 

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

if __name__ == "__main__":
    template = cv2.imread("D:\PWR\Semestr6\Projekt\datatemplate.png")
    path = "D:\PWR\Semestr6\Projekt\LSDataset"
    
    for i in glob.glob(path + '\*.png',recursive = True):
        image = cv2.imread(i)
        cv2.imshow('',image)
        cv2.waitKey()

        match = ls_match(image,template)
        if get_threshold(match) >= 0.5:
            print("True")
        else:
            print("False")