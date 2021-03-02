import pcl
import cv2
import math
import struct
import numpy as np
import time 

def getImageArrayFast(cloud):
    my_array = cloud.to_array()
    rgb_arr = my_array[:, 3]
    getb = lambda t: struct.pack('>f', t)[-1]
    vfunc = np.vectorize(getb)
    b_arr = vfunc(rgb_arr)
    tmp = np.reshape(b_arr, (int(cloud.size/cloud.width), cloud.width))
    result = np.repeat(tmp[:, :, np.newaxis], 3, axis=2)
    return result


def getImageArray(cloud):
    my_array = cloud.to_array()

    out = np.zeros(shape=(cloud.size, 1))

    for i in range(0, cloud.size):
        # cast float32 to int so that bitwise operations are possible
        out[i] = struct.pack('>f' ,my_array[i][3])[-1]

    tmp = np.reshape(out, (int(cloud.size/cloud.width), cloud.width))
    result = np.repeat(tmp[:, :, np.newaxis], 3, axis=2)

    return result


def getNormolizedZ(inpath):
    inpath = '/home/conicacui/dev_ws/src/hkclr_smart_grasping_platform/grasping_platform/data/ply/depallet-01.pcd'
    cloud = pcl.load(ply_path)
    print(type(cloud))

    height = cloud.size/cloud.width
    print(cloud.size)
    print(cloud.width, height)

    my_array = cloud.to_array()
    print(my_array.shape)

    Z = my_array[:,2]
    print(Z.shape)

    zNorm = (Z - Z.min()) / (Z.max() - Z.min()) * 255
    zNormUint8 = zNorm.astype(np.uint8)
    print(zNormUint8[0])
    print(zNormUint8.size)
    tmp = np.reshape(zNormUint8, (height, cloud.width))
    print(tmp.shape)

    b = np.repeat(tmp[:, :, np.newaxis], 3, axis=2)
    print(b.shape)

    cv2.imshow("result", b) 
    
    #waits for user to press any key  
    #(this is necessary to avoid Python kernel form crashing) 
    cv2.waitKey(0)  
    
    #closing all open windows  
    cv2.destroyAllWindows()  


def devide1000(inpath, outpath):
    ply_path = inpath
    cloud = pcl.load(ply_path)
    print(type(cloud))

    print(cloud.size)
    print(cloud.width, cloud.size/cloud.width)

    my_array = cloud.to_array()
    print(my_array.shape)
    print(my_array[1000000][0], my_array[1000000][1], my_array[1000000][2])
    new_array = my_array/1000
    print(new_array.shape)
    print(new_array[1000000][0], new_array[1000000][1], new_array[1000000][2])

    new_cloud = pcl.PointCloud(new_array)
    print(new_cloud.size)
    print(new_cloud.width)
    print(new_cloud.size/new_cloud.width)

    pcl.save(new_cloud, outpath)


ply_path = '/home/conicacui/dev_ws/src/hkclr_smart_grasping_platform/grasping_platform/data/ply/depallet-01.pcd'
cloud = pcl.load_XYZRGB(ply_path)

print(time.time(), "getImageArray start")

img = getImageArray(cloud)
print(time.time(), "getImageArray finished")

cv2.imwrite("result.jpg", img)

print(time.time(), "getImageArrayFast start")

img = getImageArrayFast(cloud)
print(time.time(), "getImageArrayFast finished")

cv2.imwrite("result2.jpg", img)