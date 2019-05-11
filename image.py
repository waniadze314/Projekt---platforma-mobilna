import time
import cv2

class imagecapture:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)

    def __del__(self):
        del(self.camera)

    def take_images(self,delay,number,path):
        image_index = 0

        while image_index < number:
            _, frame = self.camera.read()
            cv2.imwrite(path + 'data' + str(image_index) + '.png', frame)
            time.sleep(delay)
            image_index += 1

    def show_frame(self):
        _, frame = self.camera.read()
        cv2.imshow("Current Frame",frame)
        cv2.waitKey()

    def capture_frame(self):
        _,frame = self.camera.read()
        return frame