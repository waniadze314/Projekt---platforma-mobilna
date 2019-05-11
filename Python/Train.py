import sys
import mxnet as mx
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mxnet import gluon 

from classificator import *
from reader import *

if __name__ == '__main__':
    data,labels = get_data('D:\PWR\Semestr6\Projekt\ClassG3')
    dataset = get_gluon_dataset(data,labels)
    train_iter = get_data_loader(dataset,1)

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