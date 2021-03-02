#pragma once

#include "sensing/CameraController.hpp"

class FakeCameraController : public CameraController {
public:
  bool connect(std::string serial_no, RegionOfInterest roi) override;
  bool disconnect(std::string serial_no) override;
  bool getPointCloud(PointCloudColor::Ptr cloud_ptr, cv::Mat &color_mat) override;
};
