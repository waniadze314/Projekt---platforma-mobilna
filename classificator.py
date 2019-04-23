import sys
sys.path.insert(0,'..')

import numpy as np
import pandas as pd
import mxnet as mx
from mxnet import nd
from mxnet import autograd
from mxnet import init
from mxnet import gluon
from mxnet import image
from mxnet import contrib
from mxnet.gluon import nn
from mxnet.gluon import data as gluon_data
from mxnet.gluon import loss as gluon_loss
from mxnet.gluon import utils as gluon_utils

import d2l

class detector(nn.Block):
    def __init__(self):
        super(detector,self).__init__(**kwargs)
        
        self.net = nn.Sequential()
        self.net.add(nn.Dense(256,activation='relu'))
        
        self.net.initialize(init.Normal(sigma=0.1))
    
    def forward(self, x):
        for block in self._children.values():
            x = block(x)

        return x

    def save_params(self):
        self.net.save_parameters('classificator.params')