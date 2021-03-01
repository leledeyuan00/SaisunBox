#include "sensing/SmartEyeCameraController.hpp"


bool SmartEyeCameraController::connect(std::string serial_no, RegionOfInterest roi) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SmartEyeCameraController connect: %s", serial_no.c_str());

  if (SE_STATUS_SUCCESS == controller_.initDevice()) {
    float minZ = roi.z_offset;
    float maxZ = roi.z_offset + roi.depth;
    float minX = roi.x_offset;
    float maxX = roi.x_offset+ roi.width;
    float minY = roi.y_offset;
    float maxY = roi.y_offset + roi.height;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-----------------------------Init device succeed");

    // Setxy after Setz since Setz will auto compute and set maximum xy range in that z. We can use setxy to focus the ROI.
    controller_.setZRange(minZ, maxZ);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-----------------------------set z succeed");

    controller_.getZRange(minZ, maxZ);  // in millimeter
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Default depth range: %.3f, %.3f mm ", minZ, maxZ);
    
    controller_.setXYRange(minX, maxX, minY, maxY);
    controller_.getXYRange(minX, maxX, minY, maxY);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Default XY range: %.3f, %.3f, %.3f, %.3f mm ", minX, maxX, minY, maxY);
    
    //box
    // int ExposureTime2D = 8000;
    // int ExposureTime3D = 10000;

    //palletizing
    // int ExposureTime2D = 5000;
    // int ExposureTime3D = 3000;
    int ExposureTime2D = 6408;
    int ExposureTime3D = 15000;
    float MaxCoeff = 0.95;
    
    controller_.setExposureTime2D(ExposureTime2D);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExposureTime2D is set to: %d ms ", ExposureTime2D);
    
    controller_.setExposureTime3D(ExposureTime3D);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ExposureTime3D is set to: %d ms ", ExposureTime3D);
    
    controller_.setMaxCoeff(MaxCoeff);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MaxCoeff is set to: %.2f ", MaxCoeff);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized depth range: %.3f, %.3f m ", minZ * 0.001, maxZ * 0.001);

    init_ = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialize device succeed. ");
  } else {
    init_ = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialize device failed. ");
  }

  return init_;
}

bool SmartEyeCameraController::disconnect(std::string serial_no) {
  return true;
}

bool SmartEyeCameraController::getPointCloud(PointCloudColor::Ptr cloud_ptr) {
  if (!init_) {
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start capture! ");

  controller_.captureThreeModel();
  PointCloud_SE_Ptr pointCloud;
  controller_.getPointCloud(pointCloud);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture finished! ");

  if (!convert2PCLPointCloud(pointCloud, cloud_ptr)) {
    return false;
  }

  return true;
}

bool SmartEyeCameraController::convert2PCLPointCloud(const PointCloud_SE_Ptr se_cloud,
                                    const PointCloudColor::Ptr pcl_cloud) {
                                      
  if (nullptr == se_cloud || nullptr == pcl_cloud) {
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "convert2PCLPointCloud start");

  pcl_cloud->points.clear();
  int numOfPoints = se_cloud->points.size();
  for (int i = 0; i < numOfPoints; i++) {
    pcl::PointXYZRGB point;
    point.x = se_cloud->points[i].x;
    point.y = se_cloud->points[i].y;
    point.z = se_cloud->points[i].z;
    point.rgb = se_cloud->points[i].rgb;
    point.r = se_cloud->points[i].r;
    point.g = se_cloud->points[i].g;
    point.b = se_cloud->points[i].b;

    pcl_cloud->points.push_back(point);
  }

  pcl_cloud->width = se_cloud->width;
  pcl_cloud->height = se_cloud->height;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "convert2PCLPointCloud finished");

  return true;
}



