import rclpy
from rclpy.node import Node

from detect_interface import *
from saisun_msgs.srv import VisionAlgorithm
from cv_bridge import CvBridge
from cv2 import *
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.msg._point_cloud2 as pc2
from geometry_msgs.msg import Pose
import numpy as np
from read_points import *

from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import pcl

pylist = [[0.0, 0.1, 0.2],
          [1.0, 1.1, 1.2],
          [2.0, 2.1, 2.2],
          [3.0, 3.1, 3.2],
          [4.0, np.nan, 4.2]]
points = np.array(pylist, dtype=np.float32)

fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

itemsize = points.itemsize
data = points.tobytes()

# 3D (xyz) point cloud (nx3)
pcd = PointCloud2(
    header=Header(frame_id='frame'),
    height=1,
    width=points.shape[0],
    is_dense=False,
    is_bigendian=False,  # Not sure how to properly determine this.
    fields=fields,
    point_step=(itemsize * 3),  # Every point consists of three float32s.
    row_step=(itemsize * 3 * points.shape[0]),
    data=data
)

class MinimalService(Node):

    def __init__(self):
        super().__init__('vision_server')
        self.z_min_ = 0.6
        self.z_max_ = 1.6

        self.object_ = Pose()
        self.image_ = Image()
        self.point_cloud_ = PointCloud2()
        srv = VisionAlgorithm.Request()
        self.srv = self.create_service(VisionAlgorithm,'object_pose_srv',self.object_pose_callback)

    def object_pose_callback(self,request,response):
        self.image_ = request.img
        self.point_cloud_ = request.pcl
        print("======================================image =====================")
        # print(self.image_)
        print("======================================pcl========================")
        # print(" type of pcl data is : "+ str(self.point_cloud_.data))
        # print(" type of pcd data is : "+ str(pcd.data))
        
        pc_gen = read_points(self.point_cloud_,skip_nans=True)
        # pc_list = read_points(self.point_cloud_,skip_nans=True,field_names=("x","y","z"))
        pc_list = []
        print("pc_list type is :" + str(type(pc_list)))
        for p in pc_gen:
            pc_list.append([p[0],p[1],p[2],p[3]])
        # points = np.array(pc_list,dtype=)
        # pc_list = []
        # for p in pc:
        #     pc_list.append([p[0],p[1],p[2]])
        # p = pcl.PointCloud()
        # p = PointCloud2()
        
        # p.data = np.array(list(pc_list))
        # p.from_list(pc_list)
        # seg = p.make_segmenter()
        # seg.set_model_type(pcl.SACMODEL_PLANE)
        # seg.set_method_type(pcl.SAC_RANSAC)
        # indices, model = seg.segment()
        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(pc_list)
        cloud_np = pcl_data.to_array()

        

        # print("cloud_np type is :" + str(type(cloud_np)))
        print("======================================start detect========================")
        bridge = CvBridge()
        print("image width " + str(self.image_.width))
        print("image height " + str(self.image_.height))
        # cv_image = bridge.imgmsg_to_cv2(self.image_,desired_encoding='passthrough')
        im = np.frombuffer(self.image_.data, dtype=np.uint8).reshape(self.image_.height, self.image_.width, -1)

        # img_np = np.fromarrays(cv_image)
        # img_np.astype(int)

        detect_result = detectWithImg(cloud_np,im,self.z_min_,self.z_max_)

        response.success = detect_result[0]

        self.object_.position.x = detect_result[1]
        self.object_.position.y = detect_result[2]
        self.object_.position.z = detect_result[3]
        self.object_.orientation.x = detect_result[4]
        self.object_.orientation.y = detect_result[5]
        self.object_.orientation.z = detect_result[6]
        self.object_.orientation.w = detect_result[7]

        response.object = self.object_
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()