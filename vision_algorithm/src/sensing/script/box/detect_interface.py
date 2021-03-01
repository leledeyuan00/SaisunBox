import stack_detection
import box_detection
import pcl
import time
from config import parser
import numpy as np
import sys

params = parser.parse_args()

def detect(np_cloud, z_min, z_max, width, height):
    print("test")

    if(np_cloud.shape[1] == 8):
        np_cloud = np.delete(np_cloud, 3, 1)
        np_cloud = np.delete(np_cloud, 4, 1)
        np_cloud = np.delete(np_cloud, 4, 1)
        np_cloud = np.delete(np_cloud, 4, 1)

    print(np_cloud.shape)
    type = params.type
    # if(type == 0):
    #     #box
    #     return box_detection.detect(np_cloud, z_min, z_max, width, height)
    # elif(type == 1):
    #     return stack_detection.detect(np_cloud, z_min, z_max, width, height)

def testStack():
    print("testStack")
    ply_path = '../../../../data/ply_0.ply'
    cloud = pcl.load_XYZRGB(ply_path)
    pts = cloud.to_array()

    detect(pts, params.filter_z_max, params.filter_z_min, cloud.width, cloud.height)

def testBox():
    print("testBox")
    ply_path = '../../../../data/ply_0.ply'
    cloud = pcl.load_XYZRGB(ply_path)
    np_cloud = cloud.to_array()
    detect(np_cloud, 1.0, 1.5, cloud.width, cloud.height)
    
if __name__ == '__main__':
    if(params.type == 1):
        testStack()
    else:
        testBox()

