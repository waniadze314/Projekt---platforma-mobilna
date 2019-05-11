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

class Recognizer(nn.HybridBlock):
    def __init__(self,**kwargs):
        super(Recognizer,self).__init__(**kwargs)
        self.ctx = mx.cpu() 

        self.net = nn.HybridSequential()
        with self.net.name_scope():
            self.net.add(nn.Conv2D(3,kernel_size=3,padding=1,strides=10,activation='relu'))
            self.net.add(nn.BatchNorm())
            self.net.add(nn.Conv2D(3,kernel_size=3,padding=1,strides=10,activation='relu'))
            self.net.add(nn.BatchNorm())
            self.net.add(nn.MaxPool2D())
            self.net.add(nn.Dropout(0.2))
            self.net.add(nn.Conv2D(3,kernel_size=3,padding=1,strides=10,activation='relu'))
            self.net.add(nn.BatchNorm())
            self.net.add(nn.Conv2D(3,kernel_size=3,padding=1,strides=10,activation='relu'))
            self.net.add(nn.BatchNorm())
            self.net.add(nn.MaxPool2D())
            self.net.add(nn.Conv2D(3,kernel_size=3,padding=1,strides=10,activation='relu'))
            self.net.add(nn.BatchNorm())
            self.net.add(nn.Conv2D(3,kernel_size=3,padding=1,strides=10,activation='relu'))
            self.net.add(nn.BatchNorm())
            self.net.add(nn.MaxPool2D())
            self.net.add(nn.Flatten())
            self.net.add(nn.Dense(3,activation='relu'))
            
            self.net.initialize(init.Xavier())

        self.net.hybridize()

        self.loss_values = [] # array for training visualisation

    # Overwrite forward pass
    def forward(self, x):
        return self.net(x)

    # train model
    def train(self,num_epochs,train_iter,loss_function,trainer,batch_size):
        total_loss = 0.0
        print("Training: ")
        for epoch in tqdm(range(1,num_epochs+1)):
            for data,label in train_iter:
                # autograd.record() switches dropout layers on
                with autograd.record():
                    y_hat = self.net(data.as_in_context(self.ctx))
                    loss_value = loss_function(y_hat,label).sum()

                total_loss += loss_value.asscalar()
                loss_value.backward()
                trainer.step(batch_size)
            
            print('epoch %d, loss %.2f' % (epoch, total_loss))
            self.loss_values.append(total_loss)
            total_loss = 0.0
