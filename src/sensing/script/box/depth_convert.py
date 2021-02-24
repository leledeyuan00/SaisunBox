import numpy as np
import numpy.ma as ma
import time


def depth2pc(depth, intrinsics):
    scale = np.array([1.0, 1.0, 1.0])
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]

    non_zero_mask = (depth > 0)
    idxs = np.where(non_zero_mask)

    z = depth[idxs[0], idxs[1]]
    z = z * scale[2]
    x = (idxs[1] - cx) * z / fx
    y = (idxs[0] - cy) * z / fy
    pts = np.stack((x, y, z), axis=1)
    return pts


# def pc2depth(intrinsics, pc, img_shape):
#     h, w = img_shape
#     depth_map = np.zeros((h, w))
#     pc_non_zero = pc[np.where(np.all(pc[:, :] != [0.0, 0.0, 0.0], axis=-1) == True)[0]]
#     fx, fy = intrinsics[0, 0], intrinsics[1, 1]
#     cx, cy = intrinsics[0, 2], intrinsics[1, 2]
#     depth = pc_non_zero[:, 2]  # point_z
#     coords_x = pc_non_zero[:, 0] / pc_non_zero[:, 2] * fx + cx
#     coords_y = pc_non_zero[:, 1] / pc_non_zero[:, 2] * fy + cy
#     coords = np.stack([coords_x, coords_y], axis=-1)
#     depth_map[coords_y, coords_x] = depth
#     return coords.astype('int32'), depth


def pc2depth(pc, img_shape):
    depth_map = pc[:, 2].reshape(img_shape)
    return depth_map


def get_edge_point_cloud(pts, intrinsics, edges):
    # start = time.time()
    edge_mask = ma.getmaskarray(ma.masked_greater_equal(edges, 0.25))
    depth_map = pc2depth(pts, edges.shape)
    masked_depth = depth_map * edge_mask
    masked_point_cloud = depth2pc(masked_depth, intrinsics)
    # print("edge point get function :", time.time() - start)
    return masked_point_cloud
