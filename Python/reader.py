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

def get_data_batch(path,size,offset):
	data = []
	for i in range(size):
		data.append(cv2.imread(path + '\data' + str(i+offset) + '.png'))
		
	data = np.stack(data)

	return data

# def get_labels(label_type,size):
# 	if label_type == True:
# 		label = 1
# 	else:
# 		label = 0
	
# 	labels = mx.nd.zeros(size,ctx=mx.cpu())

# 	for i in range(len(labels)):		
# 		labels[i] = label

# 	return labels

def get_labels(label_type,size):
	if label_type == True:
		label = mx.nd.array([1,0],ctx=mx.cpu())
	else:
		label = mx.nd.array([0,1],ctx=mx.cpu())
	
	labels = mx.nd.zeros(shape=(size,2),ctx=mx.cpu())

	for i in range(labels.shape[0]):		
		labels[i] = mx.nd.array([0,1],ctx=mx.cpu())

	return labels

def get_gluon_dataset(data,labels):
	return mx.gluon.data.dataset.ArrayDataset(data,labels)

def get_data_loader(dataset,batch_size,shuffle):
	return mx.gluon.data.DataLoader(dataset,batch_size = batch_size,shuffle=shuffle)

def augment_dataset(dataset):
	...