import imutils
import cv2
import numpy as np

def ls_match(image,template):
    template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY) # set to grey
    template = cv2.Canny(template, 10, 25) # edge detection 
    (h,w) = template.shape[:2] # get width and height

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # set to grey 
    
    # resize image in loop to scan it fully
    for scale in np.linspace(0.2, 1.0, 20)[::-1]:
        resized_image = imutils.resize(image, width = int(image.shape[1] * scale))
        
        if resized_image.shape[0] < h or resized_image.shape[1] < w:
            break

        canny = cv2.Canny(resized_image, 10, 25) # detect egdes
        match = cv2.matchTemplate(canny, template, cv2.TM_CCOEFF) # match with cross correation
    
    return match

def get_threshold(match):
    detected = 0
    # loop through match image data
    for col in match:
        for row in col:
            # if cross correation is bigger than threshold
            # mark pixel as matching
            if row > 0.8: 
                detected += 1 

    # return percantage of matched pixels
    return float(detected/(match.shape[0]*match.shape[1]))