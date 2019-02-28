#!/usr/bin/env python


from __future__ import print_function

import cv2 as cv
import numpy as np

if __name__ == '__main__':
    print (cv.__version__)
    img = cv.imread('../maps/feb_map2.pgm',0)
    kernel = np.ones((3,3),np.uint8)
    dilation = cv.dilate(img,kernel,iterations = 1)
