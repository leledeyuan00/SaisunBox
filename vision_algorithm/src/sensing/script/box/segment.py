import pcl
import numpy as np
from config import parser


# function for filtering the points near to certain plane
def removing_plane_bg(cloud, plane_paras, dist2plane):
    np_pts = cloud.to_array()
    np_pts_vec = plane_paras[3:6] - np_pts
    p2p_dist = np.dot(np_pts_vec, plane_paras[0:3])
    outlier_idxs = np.where(p2p_dist > dist2plane)
    outlier_pts = np_pts[outlier_idxs]
    outlier_pts = outlier_pts.astype(np.float32)
    cloud_out = pcl.PointCloud()
    cloud_out.from_array(outlier_pts)
    return cloud_out

# function for removing the planes nearly perpendicular to the ground planes
# input_pts: np.array (N*3)
# params: parameter definition in config.py
# return: True for nearly perpendicular, False for not
def is_perp_plane(single_plane, ground_paras):
    # 1. get ground plane's normal
    # ground_paras = params.filter_ground_plane_para
    ground_normal = ground_paras[0:3]

    # 2. filter the plane who are perpendicular to the ground planes
    # 2.1. pre-process
    single_plane = single_plane[:, :3]
    pts_center = np.mean(single_plane, axis=0)
    normalized_pts = single_plane - pts_center  # normalization

    # 2.2. PCA analysis
    H = np.dot(normalized_pts.T, normalized_pts)
    eigenvectors, eigenvalues, eigenvectors_t = np.linalg.svd(H)  # H = U S V

    # 3. local projection
    plane_normal = eigenvectors[:, 2]

    # 4. computing the normal angle
    angle = np.dot(ground_normal, plane_normal)
    angle = np.arccos(angle)
    angle = 180*angle/3.141593
    # print("angle1: ", angle)
    diff = np.abs(angle - 90)
    if(diff < 20):
        # print("angle2: ", diff)
        return 1
    return 0
        

# function for scene plane segmentation
# input_pts: np.array (N*3)
# params: parameter definition in config.py
# return: scene planes
def plane_extract(input_pts, params):
    # 1. transform input_pts to pcl.PointCloud
    cloud_in = pcl.PointCloud()
    cloud_in.from_array(input_pts)
    # print('input successful...')   

    # 2. Uniform Downsampling
    uni_down_Filter = cloud_in.make_voxel_grid_filter()
    resolution = params.downsample_resolution
    uni_down_Filter.set_leaf_size(resolution, resolution, resolution)
    cloud_ds = uni_down_Filter.filter()
    # cloud_ds = cloud_in
    # np.savetxt("ds.txt", cloud_ds.to_array())

    # 3. Filtering by ROI
    val_cond = cloud_ds.make_ConditionAnd()
    val_cond.add_Comparison2('z', pcl.CythonCompareOp_Type.GT, params.filter_z_max)
    val_cond.add_Comparison2('z', pcl.CythonCompareOp_Type.LT, params.filter_z_min)
    val_cond.add_Comparison2('y', pcl.CythonCompareOp_Type.GT, params.filter_y_max)
    val_cond.add_Comparison2('y', pcl.CythonCompareOp_Type.LT, params.filter_y_min)
    val_cond_Filter = cloud_ds.make_ConditionalRemoval(val_cond)
    cloud_ds_zf = val_cond_Filter.filter()
    # np.savetxt("filter.txt", cloud_ds_zf.to_array())


    # 4. Filtering by plane_para
    # cloud_ds_zf = removing_plane_bg(cloud_ds_zf, params.filter_ground_plane_para, params.dist2plane)
    # cloud_ds_zf = removing_plane_bg(cloud_ds_zf, params.filter_slope_plane_para, params.dist2plane)
    # np.savetxt("fz.txt", cloud_ds_zf.to_array())

    # 4. filter outlier
    # sor_Filter = cloud_ds_zf.make_statistical_outlier_filter()
    # sor_Filter.set_mean_k(20)
    # sor_Filter.set_std_dev_mul_thresh(1.0)
    # cloud_ds_zf_sor = sor_Filter.filter()
    cloud_ds_zf_sor = cloud_ds_zf
    # np.savetxt("filter.txt", cloud_ds_zf_sor.to_array())

    # 5. Segmentation based on region growing
    tree = cloud_ds_zf_sor.make_kdtree()
    # segment = cloud_ds_zf_sor.make_RegionGrowing(ksearch=params.KNN_number) 
    segment = cloud_ds_zf_sor.make_RegionGrowing(searchRadius=params.Radius_clustering)
    seg_min_number = params.seg_min_number
    seg_max_number = params.seg_max_number
    segment.set_MinClusterSize(seg_min_number)
    segment.set_MaxClusterSize(seg_max_number)
    segment.set_NumberOfNeighbours(params.KNN_number)
    seg_smoothness = params.seg_smoothness
    seg_curvature = params.seg_curvature
    segment.set_SmoothnessThreshold(seg_smoothness)
    segment.set_CurvatureThreshold(seg_curvature)
    segment.set_SearchMethod(tree)
    cluster_indices = segment.Extract()

    # 6. Save all planes
    cluster_planes = []
    for j, indices in enumerate(cluster_indices):
        points = np.zeros((len(indices), 6), dtype=np.float32)
        color = np.random.rand(3)  # generate a random color for each plane cluster
        for i, indice in enumerate(indices):
            points[i][0] = cloud_ds_zf_sor[indice][0]
            points[i][1] = cloud_ds_zf_sor[indice][1]
            points[i][2] = cloud_ds_zf_sor[indice][2]
            points[i][3] = color[0]
            points[i][4] = color[1]
            points[i][5] = color[2]
        cluster_planes.append(points)

    return cluster_planes, cloud_ds_zf.to_array()
