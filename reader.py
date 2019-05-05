import sys
import glob
import cv2
import numpy as np

data = []

if len(sys.argv) > 0:
    for i in glob.glob(sys.argv[1] + '*.png', recursive=True):
        data.append(cv2.imread(i))

data = np.stack(data) # array of shape [num_images, height, width, channel]

def get_batch(data, batch_size):
    data_size = data.shape[0]
    indexes = list(range(data_size))
    np.random.shuffle(indexes)
    for i in range(0, data_size, batch_size):
        yield data[indexes[i:i+batch_size]]

print(data.shape)