#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensing/CameraController.hpp"
#include "sensing/BoxLocalizationAlgo.hpp"
#include "sensing/CameraModel.hpp"

class SensingServer {
public:
    bool config(CAMERALMODEL model, RegionOfInterest roi);
    bool senseObjectPose(PointCloudColor::Ptr cloud_ptr, cv::Mat &color_img);
    // bool senseObjectPose(geometry_msgs::msg::Pose &pose, double &width, double &height);

private:
    std::shared_ptr<CameraController> camera_controller_ptr_;
    // BoxLocalizationAlgo sensing_algo_;
};
