import imutils
import cv2
import numpy as np
#Open template and get canny
template = cv2.imread("D:\PWR\Semestr6\Projekt\datatemplate.png")
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
template = cv2.Canny(template, 10, 25)
(height, width) = template.shape[:2]
#open the main image and convert it to gray scale image
main_image = cv2.imread("D:\PWR\Semestr6\Projekt\TestDataset\LabelTrue\data10.png")
gray_image = cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
temp_found = None
for scale in np.linspace(0.2, 1.0, 20)[::-1]:
    #resize the image and store the ratio
    resized_img = imutils.resize(gray_image, width = int(gray_image.shape[1] * scale))
    ratio = gray_image.shape[1] / float(resized_img.shape[1])
    if resized_img.shape[0] < height or resized_img.shape[1] < width:
        break
    #Convert to edged image for checking
    e = cv2.Canny(resized_img, 10, 25)
    match = cv2.matchTemplate(e, template, cv2.TM_CCOEFF)
    (val_min, val_max, loc_min, loc_max) = cv2.minMaxLoc(match)
    if temp_found is None or val_max>temp_found[0]:
        temp_found = (val_max, loc_max, ratio)

#Get information from temp_found to compute x,y coordinate
(_, loc_max, r) = temp_found
(x_start, y_start) = (int(loc_max[0]), int(loc_max[1]))
(x_end, y_end) = (int((loc_max[0] + width)), int((loc_max[1] + height)))
#Draw rectangle around the template
cv2.rectangle(main_image, (x_start, y_start), (x_end, y_end), (153, 22, 0), 5)
cv2.imshow('Template Found', main_image)
cv2.waitKey(0)

# print(val_min)
# print(val_max)
# print(loc_min)
# print(loc_max)

detecting = 0
for col in match:
    for row in col:
        if row > 0.8:
            detecting += 1 

print(float(detecting/(match.shape[0]*match.shape[1])))