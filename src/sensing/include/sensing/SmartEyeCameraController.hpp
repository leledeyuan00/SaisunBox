#pragma once

#include "smarteye/ProcessController.h"
#include "sensing/CameraController.hpp"

class SmartEyeCameraController : public CameraController{

public:
  bool connect(std::string serial_no, RegionOfInterest roi) override;
  bool disconnect(std::string serial_no) override;
  bool getPointCloud(PointCloudColor::Ptr cloud_ptr) override;

private:
  ProcessController controller_;
  bool init_ = false;

  bool convert2PCLPointCloud(const PointCloud_SE_Ptr se_cloud, const PointCloudColor::Ptr pcl_cloud);
};
