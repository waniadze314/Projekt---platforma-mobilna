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
from tqdm import tqdm
import matplotlib.pyplot as plt

from image import *
from classificator import *

if __name__ == "__main__":
    camera = imagecapture()

    frame = camera.capture_frame()
    