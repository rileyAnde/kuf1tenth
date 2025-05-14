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

    def make(self, scan, speed):
        img = np.zeros((self.width, self.width, 1), dtype=np.uint8)
        length = len(scan)
        theta = math.radians(135.0) #(3 * pi) / 4
        deg_scan = 0.5
        loops = 0

        bubble_theta = 0

        # noise = np.random.normal(0, 0.5, scan.shape)
        # scan = scan + noise

        center_x = self.width // 2
        center_y = (self.width // 4) * 3

        for i in scan:
            if 10 >= i >= 0:
                x = int(center_x + i*5 * math.cos(theta))
                y = int(center_y + i*5 * math.sin(theta))

                if 0 <= x < self.width and 0 <= y < self.width:
                    img[y, x] = 255
            
            theta += math.radians(deg_scan)
            #print(math.degrees(theta))
        
        #BUBBLE LOGIC
        # for i in range(361):
        #     x = int(center_x + speed * math.cos(bubble_theta))
        #     y = int(center_y + speed * math.sin(bubble_theta))

        #     img[y, x] = 255

        #     bubble_theta += math.radians(1)


        cv2.imshow('wtf', img)
        cv2.waitKey(0)
        # print(f'number of loops:{loops}')
        # print(f'number of scans: {length}')
        return img

    def beprint(self, inp):
        print(inp)
# with open("kuf1tenth/f4_data.pkl", "rb") as f:
#      raw_data = pickle.load(f)
    
#      scan1 = raw_data[0][900]

# birds = Birdseye(100)
# cv2.imshow('proof', birds.make(scan1))
# cv2.waitKey(0)