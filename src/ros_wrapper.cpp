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
        use_net_sequence_ = true;
    }
    if (!nh_.getParam("algorithm_version", algorithm_version_))
        { ROS_ERROR("No algorithm_version param");
         algorithm_version_ = "0.1"; 
    }

    robot_pose_pub_ = nh_.advertise<geometry_msgs::Twist>("/saisun_pose", 10);
    robot_pose_init_ = false;
}

void SaisunWrapper::init(void)
{
    saisunState_.reset(new SaisunState(host_,port_));
    saisunState_->set_use_net_sequence(use_net_sequence_);
    saisunState_->set_algorithm_version(algorithm_version_);

    saisunCom_ = saisunState_->saisunCom_;

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

void SaisunWrapper::parse_robot_pos(void)
{
    std::vector<float> robot_pos;
    robot_pos.resize(6);
    saisunState_->get_robot_state(robot_pos);

    robot_pos_.linear.x  = (double)robot_pos[0];
    robot_pos_.linear.y  = (double)robot_pos[1];
    robot_pos_.linear.z  = (double)robot_pos[2];
    robot_pos_.angular.x = (double)robot_pos[3];
    robot_pos_.angular.y = (double)robot_pos[4];
    robot_pos_.angular.z = (double)robot_pos[5];

    robot_pose_init_ = true;
}

void SaisunWrapper::publish_msgs(void)
{
    robot_pose_pub_.publish(robot_pos_);
}

void SaisunWrapper::action_start(void)
{
    switch (receive_type_)
    {
    case GET_INIT:
    {

        break;
    }
    case GET_TRIG:
    {
        break;
    }
    case GET_DATA:
    {
        break;
    }
    case GET_POSE:
    {
        break;
    }
    
    default:
        break;
    }
}

void SaisunWrapper::control_loop(void)
{
    uint8_t msg_body[8];
    while (ros::ok())
    {
        bzero(msg_body,8);
        // robot state
        
        // cmd
        if(saisunState_->get_robot_cmd_(receive_type_,msg_body))
        {
            action_start();
        }

        // pub
        if(robot_pose_init_){
            publish_msgs();
        }
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