import time
import cv2

camera = cv2.VideoCapture(0)
delay_time = 0.1
image_index = 0

if __name__ == "__main__":
    while image_index < 100000:
        return_value, image = camera.read()
        cv2.imwrite('Images/data' + str(image_index) + '.png', image)
        time.sleep(delay_time)
        image_index += 1

    del(camera)