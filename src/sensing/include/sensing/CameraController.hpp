#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensing/RegionOfInterest.hpp"
#include "sensing/Utils.hpp"


class CameraController {
public:
  virtual bool connect(std::string serial_no, RegionOfInterest roi) = 0;
  virtual bool disconnect(std::string serial_no) = 0;
  virtual bool getPointCloud(PointCloudColor::Ptr cloud_ptr) = 0;

};
