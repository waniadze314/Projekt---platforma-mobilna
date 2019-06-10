import sys
sys.path.insert(0,'..')

import os
import cv2
import pandas as pd
import shutil
import time
import datetime
from tqdm import tqdm
import matplotlib.pyplot as plt
import glob

from image import *
from LSRecognizer import *

if __name__ == "__main__":
    template = cv2.imread("D:\PWR\Semestr6\Projekt\datatemplate.png")
    image = cv2.imread("D:\PWR\Semestr6\Projekt\LSDataset\data323.png")

    match = ls_match(image,template)
    print(get_threshold(match))