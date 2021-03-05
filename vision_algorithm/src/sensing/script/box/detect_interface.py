import stack_detection
import box_detection
import pcl
import time
from config import parser
import numpy as np
import sys
import cv2
import pcl2_img

params = parser.parse_args()

def detect(np_cloud, z_min, z_max, width, height):
    if(np_cloud.shape[1] == 8):
        np_cloud = np.delete(np_cloud, 3, 1)
        np_cloud = np.delete(np_cloud, 4, 1)
        np_cloud = np.delete(np_cloud, 4, 1)
        np_cloud = np.delete(np_cloud, 4, 1)

    print(np_cloud.shape)
    type = params.type
    if(type == 0):
        #box
        return box_detection.detect(np_cloud, z_min, z_max)
    elif(type == 1):
        img = pcl2_img.getImageArray(np_cloud, cloud.width, cloud.height)
        return stack_detection.detect(np_cloud, img)

def detectWithImg(np_cloud, img, z_min, z_max):
    if(np_cloud.shape[1] == 8):
        np_cloud = np.delete(np_cloud, 3, 1)
        np_cloud = np.delete(np_cloud, 4, 1)
        np_cloud = np.delete(np_cloud, 4, 1)
        np_cloud = np.delete(np_cloud, 4, 1)

    type = params.type
    print("type:", type)
    print("img is null?")
    print(img)
    # cv2.imwrite("./test_for_img.png",img)

    if(type == 0):
        return box_detection.detect(np_cloud, z_min, z_max)
    elif(type == 1):
        # cv2.imwrite("image_raw.jpg",img)
        print("img convert before")
        img = pcl2_img.rgb2gray(img)
        print("img convert finished")
        return stack_detection.detect(np_cloud, img)

def testStack(np_cloud, img, width, height):
    print("testStack")
    detectWithImg(np_cloud, img, params.filter_z_max, params.filter_z_min)
    #detect(np_cloud, params.filter_z_max, params.filter_z_min, width, height)

def testBox(np_cloud, img):
    print("testBox")
    detectWithImg(np_cloud, img, 1.0, 1.5)

    
if __name__ == '__main__':
    ply_path = '../../../../data/ply_10.ply'
    cloud = pcl.load_XYZRGB(ply_path)
    np_cloud = cloud.to_array()

    img1 = cv2.imread('../../../../data/rgb.jpg')
    # img1 = pcl2_img.rgb2gray(img1)

    time1 = time.time()
    if(params.type == 1):
        testStack(np_cloud, img1, cloud.width, cloud.height)
    else:
        testBox(np_cloud, img1)
    print('total time: ', time.time() - time1)


