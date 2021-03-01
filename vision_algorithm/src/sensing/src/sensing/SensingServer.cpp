#include "sensing/SensingServer.hpp"
#include "sensing/FakeCameraController.hpp"
#include "sensing/SmartEyeCameraController.hpp"
#include <pcl/io/ply_io.h>

bool SensingServer::config(CAMERALMODEL model, RegionOfInterest roi) {
    switch (model)
    {
    case CAMERALMODEL::FAKE_CAMERA:
      camera_controller_ptr_ = std::make_shared<FakeCameraController>();
      break;
    
    case CAMERALMODEL::SMARTEYE_HV1000:
      camera_controller_ptr_ = std::make_shared<SmartEyeCameraController>();
      break;

    default:
      return false;
    }

    camera_controller_ptr_->connect("", roi);
    sensing_algo_.config(roi);
    return true;
}

bool SensingServer::senseObjectPose(geometry_msgs::msg::Pose &pose, double &width, double &height) {
    PointCloudColor::Ptr cloud_ptr(new PointCloudColor);
    if(!camera_controller_ptr_->getPointCloud(cloud_ptr)){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Get point cloud failed!");
        return false;
    }
    // test for save pcl
    std::string file_name = "ply_";
    file_name.append(std::to_string(10));
    file_name.append(".ply");
    pcl::io::savePLYFile(file_name,*cloud_ptr);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get point cloud succeed!");
    return sensing_algo_.getObjectPose(cloud_ptr, pose, width, height);
}



