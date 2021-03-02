import stack_detection
import pcl
import time
import cv2
import pcl2_img

from visualize import *
import open3d as o3d

# main program
if __name__ == '__main__':
    ply_path = '../../../../data/ply_0.ply'

    cloud = pcl.load_XYZRGB(ply_path)
    pts = cloud.to_array()

    img = cv2.imread('../../../../data/img.jpg')
    img = pcl2_img.rgb2gray(img)

    time1 = time.time()
    success, result_tuple = stack_detection.detect_with_view(pts, img)
    if(success):
        result, grasp_box, cluster_planes, idx = result_tuple

        R, t, pose, rec_wid, rec_len = result
        visualize_scene_with_pose(cluster_planes, idx, R, t)

        print(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len)

        # 3.7 visulization in the scene
        print('total time: ', time.time() - time1)

        pts = pts[:, 0:3]
        scene = o3d.geometry.PointCloud()
        scene.points = o3d.utility.Vector3dVector(pts)
        visualize_pose_in_raw_pts(scene, grasp_box[:, 0:3], R, t)
    else:
        print('No plane')
