#pragma once

#include <Python.h>
#include <geometry_msgs/msg/pose.hpp>
#include "sensing/Utils.hpp"
#include "sensing/RegionOfInterest.hpp"
#include <opencv2/core/mat.hpp>

class BoxLocalizationAlgo {
public:
    BoxLocalizationAlgo();
    ~BoxLocalizationAlgo();

    void config(RegionOfInterest roi);
    bool getObjectPose(PointCloudColor::Ptr cloud_ptr, cv::Mat color_img, geometry_msgs::msg::Pose &pose, double &width, double &height);
   
private:
    RegionOfInterest roi_;
    PyObject *pyModule_;
    PyObject *pyFunc_;

    bool run_once_;

    int init();
};
