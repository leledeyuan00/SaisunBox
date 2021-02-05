#include "saisun_driver/ros_wrapper.h"

SaisunWrapper::SaisunWrapper(ros::NodeHandle &nh):nh_(nh)
{
    ros_init();
    init();
}

void SaisunWrapper::ros_init(void)
{
    // param get
    if (!nh_.getParam("saisun_ip", host_))
        { ROS_ERROR("No saisun_ip param"); 
        host_ = "172.16.1.30";
    }
    int port;
    if (!nh_.getParam("saisun_port", port))
        { ROS_ERROR("No saisun_port param"); 
        port_ = 9999;
    }
    port_ = port;
    if (!nh_.getParam("use_net_sequence", use_net_sequence_))
        { ROS_ERROR("No use_net_sequence param"); 
        use_net_sequence_ = false;
    }
    if (!nh_.getParam("algorithm_version", algorithm_version_))
        { ROS_ERROR("No algorithm_version param");
         algorithm_version_ = "0.1"; 
    }
}

void SaisunWrapper::init(void)
{
    saisunState_.reset(new SaisunState(host_,port_));
    saisunState_->set_use_net_sequence(use_net_sequence_);
    saisunState_->set_algorithm_version(algorithm_version_);

    control_loop_thread_.reset(new std::thread(
        boost::bind(&SaisunWrapper::control_loop,this)));
}

void SaisunWrapper::start(void)
{
    saisunState_->start();
}

void SaisunWrapper::halt(void)
{
    saisunState_->halt();
    control_loop_thread_->join();
}

void SaisunWrapper::control_loop(void)
{
    while (ros::ok())
    {
        saisunState_->get_robot_state();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "saisun_vision_node");
    ros::NodeHandle nh;

    SaisunWrapper saisun_wrapper(nh);
    saisun_wrapper.start();

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();

    saisun_wrapper.halt();

    return 0;    
}