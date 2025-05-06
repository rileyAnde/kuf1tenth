'''
RILEY ANDERSON
'''
import cv2
import numpy as np
import math
from math import pi
import pickle


class Birdseye():
    def __init__(self, width):
        self.width = width

    def make(self, scan):
        img = np.zeros((self.width, self.width, 1), dtype=np.uint8)
        length = len(scan)
        theta = (3 * pi) / 4
        deg_scan = 270/length

        center_x = self.width // 2
        center_y = (self.width // 4) * 3

        for i in scan:
            if 10 >= i >= 0:
                x = int(center_x + i*10 * math.cos(theta))
                y = int(center_y + i*10 * math.sin(theta))

                if 0 <= x < self.width and 0 <= y < self.width:
                    img[y, x] = 255
            
            theta += math.radians(deg_scan)

        return img

# with open("joy_data2.pkl", "rb") as f:
#     raw_data = pickle.load(f)
    
#     scan1 = raw_data[0][700]

# birds = Birdseye(100)
# cv2.imshow('proof', birds.make(scan1))
# cv2.waitKey(0)