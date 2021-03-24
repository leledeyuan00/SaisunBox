#include "sensing/FakeCameraController.hpp"
#include <pcl/io/ply_io.h>
#include <opencv2/imgcodecs.hpp> 

bool FakeCameraController::connect(std::string serial_no, RegionOfInterest roi) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FakeCameraController connect: %s", serial_no.c_str()); 
  return true;
}

bool FakeCameraController::disconnect(std::string serial_no) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FakeCameraController disconnect: %s", serial_no.c_str()); 
  return true;
}

bool FakeCameraController::getPointCloud(PointCloudColor::Ptr cloud_ptr, cv::Mat &color_mat) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FakeCameraController capture!");

  std::string filename = "./ply_0.ply";
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s", filename);
    return (-1);
  }
  
  color_mat = cv::imread("./img.png", cv::IMREAD_COLOR);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fake color: %d %d %d %d %d",color_mat.empty(), color_mat.cols, color_mat.rows, color_mat.total(), color_mat.channels()); 


  return true;
}




