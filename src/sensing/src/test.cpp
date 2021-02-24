
#include "sensing/SensingServer.hpp"

int main(int argc, char **argv)
{
    SensingServer server;

    RegionOfInterest roi;
    roi.x_offset = -800.0;
    roi.y_offset = -1404.0;
    roi.z_offset = 1800.0;
    roi.width = 2216.0;
    roi.height = 2274.0;
    roi.depth = 800.0;

    server.config(CAMERALMODEL::FAKE_CAMERA, roi);

    geometry_msgs::msg::Pose pose;
    double width = 0.0;
    double height = 0.0;
    bool result = server.senseObjectPose(pose, width, height);
    
    if(result) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensing succeed!");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose: [%f %f %f %f %f %f %f]", pose.position.x, 
            pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Size: [%f %f ]", width, height); 
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Sensing failed!");
    }

    return 0;
}
