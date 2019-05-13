import sys
import glob
import cv2
import numpy as np

import mxnet as mx
from mxnet import gluon

def get_data(path):
    data = []

    for i in glob.glob(path + '\*.png', recursive=True):
        data.append(cv2.imread(i))

    data = np.stack(data) # array of shape [num_images, height, width, channel]
    
    return data

def get_lables(label_type,size):
    if label_type == True:
		label = mx.nd.array([1,0],ctx=mx.cpu())
	else:
    	label = mx.nd.array([0,1],ctx=mx.cpu())

	labels = mx.nd.zeros(shape=(size,2),ctx=mx.cpu())

	for i in range(labels.shape[0]):
    	labels[i] = mx.nd.array([0,1],ctx=mx.cpu())

	return labels

def get_batch(data, batch_size):
    data_size = data.shape[0]
    indexes = list(range(data_size))
    np.random.shuffle(indexes)
    for i in range(0, data_size, batch_size):
        yield data[indexes[i:i+batch_size]]

def get_gluon_dataset(data,labels):
    return mx.gluon.data.dataset.ArrayDataset(data,labels)

def get_data_loader(dataset,batch_size):
    return mx.gluon.data.DataLoader(dataset,batch_size = batch_size)

def augment_dataset(dataset):
    ...