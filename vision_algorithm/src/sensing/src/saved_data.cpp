#include "sensing/visibility_control.h"
#include "sensing/SensingServer.hpp"

#include <functional>
#include <memory>
#include <thread>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"

#include <stdlib.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>

class SavedData : public rclcpp::Node
{
public:
    SavedData();
    void captureSave(void);

private:
    SensingServer sensing_server_;
    RegionOfInterest roi_;
    cv::Mat color_mat_;
    PointCloudColor::Ptr cloud_ptr_;

    size_t data_count_;

    std::string home_dir_;

    // std::ofstream outfile_;
};

SavedData::SavedData():Node("saved_data"), data_count_(1)
{
    roi_.x_offset = -3000;
    roi_.y_offset = -3000;
    roi_.z_offset = 1500;
    roi_.width = 6000;
    roi_.height = 6000;
    roi_.depth = 1670;

    cloud_ptr_.reset(new PointCloudColor());
    sensing_server_.config(CAMERALMODEL::SMARTEYE_HV1000,roi_);

    home_dir_ = getenv("HOME");

}

void SavedData::captureSave(void)
{
    sensing_server_.senseObjectPose(cloud_ptr_,color_mat_);


    std::stringstream pcl_file_name;
    std::stringstream img_file_name;

    pcl_file_name << home_dir_ << "/lg/data"+ std::to_string(data_count_) +"/ply/";
    img_file_name << home_dir_ << "/lg/data"+ std::to_string(data_count_) +"/img/";

    std::string mkdir_("mkdir -p ");
    
    std::system((mkdir_+ pcl_file_name.str()).c_str());
    std::system((mkdir_+ img_file_name.str()).c_str());

    std::time_t now = std::time(NULL);
    std::tm *lt = std::localtime(&now);

    std::stringstream tim_dir;
    tim_dir << lt->tm_hour << "_" << lt->tm_min << "_" << lt->tm_sec;

    pcl_file_name << tim_dir.str() << ".ply";
    img_file_name << tim_dir.str() << ".png";

    // outfile_.open(pcl_file_name.str());
    // if(!outfile_)
    // {
    //     RCLCPP_INFO(this->get_logger(),"saved file failed");
    //     return;
    // }
    //     outfile_ << "something" <<std::endl;
    //     outfile_.close();
    RCLCPP_INFO(this->get_logger(),"Saved point cloud is %d", pcl::io::savePLYFile(pcl_file_name.str(),*cloud_ptr_));
    if(!cv::imwrite(img_file_name.str(),color_mat_))
        RCLCPP_INFO(this->get_logger(),"Saved img failed!!");    
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);

    SavedData saved_data_;

    int fd_ = 0;
    char c;

    while (rclcpp::ok())
    {
        read(fd_,&c,1);
        printf("input is: %d",c);
        if(c == 10 && rclcpp::ok()){
            saved_data_.captureSave();
        }
    }
    return 0;
}

