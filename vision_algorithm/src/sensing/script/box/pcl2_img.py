import pcl
import numpy as np
import struct
import cv2

def getb(rgb):
    b = struct.pack('>f', rgb)[-1]
    # if(b == 0):
    #     b = 255
    return b

def getImageArray(pts, width, height):
    rgb_arr = pts[:, 3]
    vfunc = np.vectorize(getb)
    b_arr = vfunc(rgb_arr)
    result = b_arr.reshape(height, width)
    return result.astype(np.uint8)

def rgb2gray(img):
    gray_img = img[:, :,0]
    #gray_img = gray_img[:, :, np.newaxis]
    cv2.imwrite("gray.jpg", gray_img)
    return gray_img