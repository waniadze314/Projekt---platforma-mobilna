import sys
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mxnet import gluon 

from Reader import *
from Net import *

if __name__ == '__main__':

    Neuralnet = NeuralNet()
    load_size = 50

    for i in range(1000/load_size):
        data_true = get_data_batch('D:\PWR\Semestr6\Projekt\Dataset\LabelTrue',load_size,load_size*i)
        labels_true = get_labels(True,data_true.shape[0])
        data_false = get_data_batch('D:\PWR\Semestr6\Projekt\Dataset\LabelFalse',load_size,load_size*i)
        labels_false = get_labels(False,data_false.shape[0])

        data_true = mx.nd.array(data_true,dtype='float32',ctx=mx.cpu())
        data_false = mx.nd.array(data_false,dtype='float32',ctx=mx.cpu())
        data = mx.nd.concat(data_true,data_false,dim=0)

        lables_true = mx.nd.array(labels_true,ctx=mx.cpu())
        labels_false = mx.nd.array(labels_false,ctx=mx.cpu())
        labels = mx.nd.concat(labels_true,labels_false,dim=0)
    
        dataset = get_gluon_dataset(data,labels)
        train_iter = get_data_loader(dataset,20)

        loss_function = gluon.loss.SoftmaxCrossEntropyLoss()
        trainer = gluon.Trainer(Neuralnet.net.collect_params(),'sgd',{'learning_rate' : 0.1, 'momentum' : 0.9, 'wd' : 5e-4})

        Neuralnet.train(5, train_iter, loss_function, trainer,20)

        del(data_true)
        del(data_false)
        del(labels_true)
        del(labels_false)
        del(dataset)
        del(train_iter)

    Neuralnet.save_parameters('Net.params')




    