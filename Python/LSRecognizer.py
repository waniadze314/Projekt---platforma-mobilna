import imutils
import cv2
import numpy as np

def ls_match(image,template):
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    template = cv2.Canny(template, 10, 25)
    (h,w) = template.shape[:2]

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    for scale in np.linspace(0.2, 1.0, 20)[::-1]:
        #resize the image and store the ratio
        resized_img = imutils.resize(image, width = int(image.shape[1] * scale))
        ratio = image.shape[1] / float(resized_img.shape[1])
        if resized_img.shape[0] < h or resized_img.shape[1] < w:
            break

        canny = cv2.Canny(resized_img, 10, 25)
        match = cv2.matchTemplate(canny, template, cv2.TM_CCOEFF)
    
    return match

def get_threshold(match):
    detecting = 0
    for col in match:
        for row in col:
            if row > 0.8:
                detecting += 1 

    return float(detecting/(match.shape[0]*match.shape[1]))
