import stack_detection
import pcl
import time
import cv2
import pcl2_img
import os

from visualize import *
import open3d as o3d

def single_file_test():
    ply_path = './ply_0.ply'

    cloud = pcl.load_XYZRGB(ply_path)
    pts = cloud.to_array()

    img = cv2.imread('./img.png')
    img = pcl2_img.rgb2gray(img)

    time1 = time.time()
    success, result_tuple = stack_detection.detect_with_view(pts, img)
    if (success):
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

def loop_file_test():
    ply_file_path = './data/SmartEye_3_10/data1/ply'
    ply_path_list = os.listdir(ply_file_path)
    ply_path_list.sort()

    img_file_path = './data/SmartEye_3_10/data1/img'
    img_path_list = os.listdir(img_file_path)
    img_path_list.sort()

    for i in range(len(ply_path_list)):
        ply_path = ply_file_path + '/' + ply_path_list[i]
        print(ply_path)
        img_path = img_file_path + '/' + img_path_list[i]
        print(img_path)

        cloud = pcl.load_XYZRGB(ply_path)
        pts = cloud.to_array()

        img = cv2.imread(img_path)
        img = pcl2_img.rgb2gray(img)

        time1 = time.time()
        success, result_tuple = stack_detection.detect_with_view(pts, img)
        if (success):
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


# main program
if __name__ == '__main__':
    single_file_test()
    # loop_file_test()
