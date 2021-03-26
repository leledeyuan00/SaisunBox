import argparse

parser = argparse.ArgumentParser()

# WUXI LAB
# segment relevant
parser.add_argument('--type', type=int, default= 1)
parser.add_argument('--model', type=str, default='model.yml')
parser.add_argument('--downsample_resolution', type=float, default=0.002, help='the resolution for pcl voxel grid filter')
parser.add_argument('--filter_z_min', type=float, default=3.1) # 1354 for 11.30 data
parser.add_argument('--filter_z_max', type=float, default=1.5) # 1000 for 11.30 data
parser.add_argument('--filter_y_min', type=float, default=1.0)  # roi in y-axis
parser.add_argument('--filter_y_max', type=float, default=-1.0) # roi in y-axis

# clustering relevant
parser.add_argument('--Radius_clustering', type=int, default=0.00475)
parser.add_argument('--KNN_number', type=int, default=10)  # 30 
parser.add_argument('--seg_min_number', type=int, default=3000)
parser.add_argument('--seg_max_number', type=int, default=200000)
parser.add_argument('--seg_smoothness', type=float, default=0.12)
parser.add_argument('--seg_curvature', type=int, default=0.025)

# filtering edge points relevant
parser.add_argument('--Radius_KD', type=int, default=0.005) 

# filtering background relevant
parser.add_argument('--filter_slope_plane_para', type=list,
                    default=[-0.021592, 0.761064, 0.648317, -0.285321, 0.463227, 2.301724])
parser.add_argument('--filter_ground_plane_para', type=list,
                    default=[-0.031156, 0.150190, 0.988166, 0.133395, 0.095292, 2.456849])  # ground plane paras
parser.add_argument('--dist2plane', type=float, default=0.02)

# gripper relevant
parser.add_argument('--grasper_size_para', type=list,
                    default=[0.28, 0.2, 0.16, 0.115])  # grasper_box_x/y/z/Sucker length

# edge detection relevant
parser.add_argument('--camera_intrinsics', type=list,
                    default=[2.3472340283994704e+03, 2.3472340283994704e+03, 799, 980])  # camera value3: smaller->left
parser.add_argument('--PREPROCESS_THRESHOLD_DEFAULT', type=int, default=30)  # threshold for edge detection

# task relevant
parser.add_argument('--UNSTACK', type=int, default=1)  # toggle for UNSTACK or BOX-PICKING
