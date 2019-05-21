import cv2
import glob
import numpy as np
from matplotlib import pyplot as plt
from tqdm import tqdm

camera = cv2.VideoCapture(0)
img = cv2.imread("D:\PWR\Semestr6\Projekt\TestDataset\LabelFalse\data0.png")
canny = cv2.Canny(img,160,100)
cv2.imshow('',canny)
cv2.waitKey()

source_path = 'D:\PWR\Semestr6\Projekt\Dataset\LabelFalse'
index = 0
for i in tqdm(glob.glob(source_path + '\*.png', recursive=True)):
    img = cv2.imread(i)
    img = cv2.Canny(img,100,80)
    cv2.imwrite('D:\PWR\Semestr6\Projekt\AugmentedDataset\Canny\LabelFalse\data' + str(index) + '.png',img)
    index += 1
