#include "sensing/SensingServer.hpp"
#include "sensing/FakeCameraController.hpp"
#include "sensing/SmartEyeCameraController.hpp"
#include <pcl/io/ply_io.h>

bool SensingServer::config(CAMERALMODEL model, RegionOfInterest roi) 
{
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
    // sensing_algo_.config(roi);
    return true;
}

bool SensingServer::senseObjectPose(PointCloudColor::Ptr cloud_ptr, cv::Mat &color_img)
{
  if(!camera_controller_ptr_->getPointCloud(cloud_ptr, color_img)){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Get point cloud failed!");
      return false;
  }
  return true;
}

bool SensingServer::senseObjectPose(geometry_msgs::msg::Pose &pose, double &width, double &height)
{
  PointCloudColor::Ptr cloud_ptr(new PointCloudColor);
  cv::Mat color_img;
  bool success;
  if(!camera_controller_ptr_->getPointCloud(cloud_ptr, color_img)){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Get point cloud failed!");
      return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get point cloud succeed!");
  try
  {
    success = sensing_algo_.getObjectPose(cloud_ptr, color_img, pose, width, height);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    success = false;
  }
  return success;
}


