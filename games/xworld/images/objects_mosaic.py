#!/usr/bin/python

import os
import glob
import cv2
import numpy as np

"""
Retrieve all the object icons and combine them into a single figure
"""

if __name__ == "__main__":
    icons = glob.glob("goal/*/*.jpg")
    icons.sort()

    ## append the wall and agent icons
    icons.append("agent/robot_1.jpg")
    icons.append("block/brick_1.jpg")

    icon_imgs = [cv2.imread(i) for i in icons]

    print(len(icon_imgs))

    size, _, _ = icon_imgs[0].shape

    Y, X = 19, 19

    height = size * Y
    width = size * X
    img = np.ones((height, width, 3), np.uint8) * 255

    for i in range(Y):
        for j in range(X):
            idx = i * X + j
            if idx < len(icon_imgs):
                img[i*size : (i+1)*size, j*size : (j+1)*size] = icon_imgs[idx].copy()

    # cv2.imshow("test", img)
    # cv2.waitKey(0)

    cv2.imwrite("/tmp/icons_all.jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
