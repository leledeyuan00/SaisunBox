#pragma once

#include <Python.h>
#include <geometry_msgs/msg/pose.hpp>
#include "sensing/Utils.hpp"
#include "sensing/RegionOfInterest.hpp"

class BoxLocalizationAlgo {
public:
    BoxLocalizationAlgo();
    ~BoxLocalizationAlgo();

    void config(RegionOfInterest roi);
    bool getObjectPose(PointCloudColor::Ptr cloud_ptr, geometry_msgs::msg::Pose &pose, double &width, double &height);
   
private:
    RegionOfInterest roi_;
    PyObject *pyModule_;
    PyObject *pyFunc_;

    int init();
};