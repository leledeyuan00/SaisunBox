import pcl
import numpy as np
import struct

# def getImageArrayFast(cloud):
#     my_array = cloud.to_array()
#     rgb_arr = my_array[:, 3]
#     getb = lambda t: struct.pack('>f', t)[-1]
#     vfunc = np.vectorize(getb)
#     b_arr = vfunc(rgb_arr)
#     tmp = np.reshape(b_arr, (int(cloud.size/cloud.width), cloud.width))
#     result = np.repeat(tmp[:, :, np.newaxis], 1, axis=2)
#     return result


def getImageArrayFast(pts, width, height):
    rgb_arr = pts[:, 3]
    getb = lambda t: struct.pack('>f', t)[-1]
    vfunc = np.vectorize(getb)
    b_arr = vfunc(rgb_arr)
    result = b_arr.reshape(height, width)
    return result.astype(np.uint8)

# load .ply/pcd and transform it into numpy array
def load_ply(ply_path):
    # pts = o3d.io.read_point_cloud(ply_path)
    # # N * 3
    # np_pts = np.array(pts.points).astype(np.float32)
    cloud = pcl.load_XYZRGB(ply_path)
    return cloud