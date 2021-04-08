import rclpy
from rclpy.node import Node

import numpy as np
from cv_bridge import CvBridge
from cv2 import *
import pcl

from detect_interface import *
from stack_detection import *
from saisun_msgs.srv import VisionAlgorithm
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.msg._point_cloud2 as pc2
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

from read_points import *
from config import parser


params = parser.parse_args()

class MinimalService(Node):

    def __init__(self):
        super().__init__('vision_server')
        self.z_min_ = 1.5
        self.z_max_ = 3.4

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
        
        # pc_gen = read_points(self.point_cloud_,skip_nans=True)
        # pc_list = []
        # print("pc_list type is :" + str(type(pc_list)))
        # for p in pc_gen:
        #     pc_list.append([p[0],p[1],p[2],p[3]])
       
        # pcl_data = pcl.PointCloud_PointXYZRGB()
        # pcl_data.from_list(pc_list)
        # cloud_np = pcl_data.to_array()

        

        print("======================================start detect========================")
        print("image width " + str(self.image_.width))
        print("image height " + str(self.image_.height))
        im = np.frombuffer(self.image_.data, dtype=np.uint8).reshape(self.image_.height, self.image_.width, -1)


        ply_path = '/home/jiang/lg/data6/ply/17_48_38.ply'

        cloud = pcl.load_XYZRGB(ply_path)
        pts = cloud.to_array()

        img = cv2.imread('/home/jiang/lg/data6/img/17_48_38.png')
        img = pcl2_img.rgb2gray(img)

        detect_result = stack_detection.detect(pts,img)
        # detect_result = detectWithImg(pts,img,self.z_min_,self.z_max_)

        # detect_result = detectWithImg(cloud_np,im,self.z_min_,self.z_max_)

        response.success = bool(detect_result[0])

        self.object_.position.x =    float(detect_result[1])
        self.object_.position.y =    float(detect_result[2])
        self.object_.position.z =    float(detect_result[3])
        self.object_.orientation.x = float(detect_result[4])
        self.object_.orientation.y = float(detect_result[5])
        self.object_.orientation.z = float(detect_result[6])
        self.object_.orientation.w = float(detect_result[7])

        response.object = self.object_
        print("detect finished")
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