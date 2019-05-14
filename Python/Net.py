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
import pandas as pd
import shutil
import time
from tqdm import tqdm

class NeuralNet(nn.HybridBlock):
    def __init__(self,**kwargs):
        super(NeuralNet,self).__init__(**kwargs)
        self.ctx = mx.cpu() 

        self.net = nn.HybridSequential()
        with self.net.name_scope():
            self.net.add(nn.Conv2D(channels=3,kernel_size=(5,3),padding=1,strides=1,activation='sigmoid'))
            self.net.add(nn.AvgPool2D(pool_size=3,strides=1))
            self.net.add(nn.Dense(8,activation='relu'))
            self.net.add(nn.Dropout(0.2))
            self.net.add(nn.Dense(1,activation='relu'))
            
            self.net.initialize(init.Normal(sigma=1))

        self.net.hybridize()

    # Overwrite forward pass
    def hybrid_forward(self, x):
        return self.net(x)

    # train model
    def train(self,num_epochs,train_iter,loss_function,trainer,batch_size):
        print("Training: ")
        for epoch in tqdm(range(1,num_epochs+1)):
            for data,label in train_iter:
                # autograd.record() switches dropout layers on
                with autograd.record():
                    y_hat = self.net(data.as_in_context(self.ctx))
                    loss_value = loss_function(y_hat,label).sum()

                loss_value.backward()
                trainer.step(batch_size)