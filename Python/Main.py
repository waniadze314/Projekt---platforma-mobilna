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

from image import *
from Net import *

if __name__ == "__main__":
    Net = NeuralNet()
    Net.load_parameters('Net.params')

    while True:
        frames = []

        camera = imagecapture()
        frame = camera.capture_frame()
        frames.append(frame)

        frameset = mx.gluon.data.dataset.ArrayDataset(mx.nd.array(frame,dtype='float32',ctx=mx.cpu()))
        frame_iter = mx.gluon.data.DataLoader(frameset,batch_size=1)

        for data in frame_iter:
            output = Net.net(data.as_in_context(Net.ctx))
            if output.asnumpy().argmax() == 0:
                f = open("times.txt","w+")
                f.write(datetime.datetime.now())
                f.close()

    
