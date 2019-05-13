import sys
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mxnet import gluon 

from classificator import *
from reader import *

if __name__ == '__main__':
    data_true = get_data('D:\PWR\Semestr6\Projekt\Dataset\LabelsTrue')
    labels_true = get_labels(True,data_true.shape[0])
    data_false = get_data('D:\PWR\Semestr6\Projekt\Dataset\Labelsfalse')
    labels_false = get_labels(False,data_false.shape[0])

    data_true = mx.nd.array(data_true,dtype='float32',ctx=mx.cpu())
    data_false = mx.nd.array(data_false,dtype='float32',ctx=mx.cpu())
    data = mx.nd.concat(data_true,data_false,dim=0)

    lables_true = mx.nd.array(labels_true,ctx=mx.cpu())
    labels_false = mx.nd.array(labels_false,ctx=mx.cpu())
    labels = mx.nd.concat(labels_true,labels_false,dim=0)

    dataset = get_gluon_dataset(data,labels)
    train_iter = get_data_loader(dataset,50)

    if len(sys.argv) == 1:
        deep_net = Recognizer()
        deep_net.load_parameters('recognizer.params')

    else:
        if sys.argv[1] == 'train':  
            deep_net = Recognizer()
         
            loss_function = gluon.loss.SoftmaxCrossEntropyLoss()
            trainer = gluon.Trainer(deep_net.net.collect_params(),'sgd',{'learning_rate' : 0.1, 'momentum' : 0.9, 'wd' : 1e-4})

            deep_net.train(10, #num_epchos
                            train_iter,    # Data iterator 
                            loss_function, # Softmax
                            trainer,
                            128) #batch_size
          
            deep_net.save_parameters('recognizer.params')

        else:
            print(sys.argv[1])

    print("Testing: ")
    accuracy = 0
    for data,label in test_iter:
        output = deep_net.net(data.as_in_context(deep_net.ctx))
        if output.asnumpy().argmax() == label.asnumpy()[0]:
            accuracy += 1
            
    result = float(accuracy/len(test_data))
    print(result*100,"%")

    # sns.regplot(deep_net.loss_values)
    # plt.show()