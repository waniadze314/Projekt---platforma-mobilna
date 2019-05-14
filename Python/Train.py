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
    load_size = 100

    for i in range(int(800/load_size)):
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
        train_iter = get_data_loader(dataset,10,True)

        # loss_function = mx.gluon.loss.SigmoidBinaryCrossEntropyLoss()
        loss_function = mx.gluon.loss.L2Loss()
        trainer = gluon.Trainer(Neuralnet.net.collect_params(),'sgd',{'learning_rate' : 1e-6 , 'momentum' : 0.8, 'wd' : 1e-8})

        Neuralnet.train(4, train_iter, loss_function, trainer,50)

        del(data_true)
        del(data_false)
        del(labels_true)
        del(labels_false)
        del(dataset)
        del(train_iter)

    #Neuralnet.load_parameters('Net.params')

    data_true_test = get_data_batch('D:\PWR\Semestr6\Projekt\TestDataset\LabelTrue',10,0)
    labels_true_test = get_labels(True,data_true_test.shape[0])
    data_false_test = get_data_batch('D:\PWR\Semestr6\Projekt\TestDataset\LabelFalse',10,0)
    labels_false_test = get_labels(False,data_false_test.shape[0])

    data_true_test = mx.nd.array(data_true_test,dtype='float32',ctx=mx.cpu())
    data_false_test = mx.nd.array(data_false_test,dtype='float32',ctx=mx.cpu())
    data_test = mx.nd.concat(data_true_test,data_false_test,dim=0)

    lables_true_test = mx.nd.array(labels_true_test,dtype='float32',ctx=mx.cpu())
    labels_false_test = mx.nd.array(labels_false_test,dtype='float32',ctx=mx.cpu())
    labels_test = mx.nd.concat(labels_true_test,labels_false_test,dim=0)

    test_dataset = get_gluon_dataset(data_test,labels_test)
    test_iter = get_data_loader(test_dataset,1,True)

    #loss = mx.gluon.loss.L2Loss()
    #trainer = gluon.Trainer(Neuralnet.net.collect_params(),'sgd',{'learning_rate' : 1e-8, 'momentum' : 0.8 })

    #Neuralnet.train(10,test_iter,loss,trainer,1)

    print("Testing: ")
    accuracy = 0
    for data,label in test_iter:
        output = Neuralnet.net(data.as_in_context(Neuralnet.ctx))
        print("out",output.asnumpy())
        print("label",label.asnumpy())

        if abs(label.asnumpy()[0] - output.asnumpy()[0]) < 0.5:
            accuracy += 1 
            
    result = float(accuracy/len(data_test))
    print(result*100,"%")

    print("Saving:")
    Neuralnet.save_parameters('P.params')