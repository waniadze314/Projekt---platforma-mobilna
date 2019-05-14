from PIL import Image

def count_pixels(image):
    green = 0
    for pixel in image.getdata():
        if pixel == (0,1,0):
            green += 1

    return green
