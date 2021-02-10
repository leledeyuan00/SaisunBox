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

    robot_pose_pub_ = nh_.advertise<geometry_msgs::Twist>("/saisun_pose", 10);
    
    saisun_init_ac_.reset(new actionlib::SimpleActionClient<saisun_msgs::InitialAction>("saisun_robot_initial",true));
    saisun_trig_ac_.reset(new actionlib::SimpleActionClient<saisun_msgs::TriggerAction>("saisun_robot_trigger",true));
    saisun_result_ac_.reset(new actionlib::SimpleActionClient<saisun_msgs::GetResultAction>("saisun_robot_get_result",true));
    
    robot_pose_init_ = false;
}

void SaisunWrapper::init(void)
{
    saisunState_.reset(new SaisunState(host_,port_));
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
    
    robot_pose_init_ = saisunState_->get_robot_pose(robot_pos);

    robot_pos_.linear.x  = (double)robot_pos[0];
    robot_pos_.linear.y  = (double)robot_pos[1];
    robot_pos_.linear.z  = (double)robot_pos[2];
    robot_pos_.angular.x = (double)robot_pos[3];
    robot_pos_.angular.y = (double)robot_pos[4];
    robot_pos_.angular.z = (double)robot_pos[5];

}

void SaisunWrapper::publish_msgs(void)
{
    robot_pose_pub_.publish(robot_pos_);
}

// action callback
void SaisunWrapper::initial_ac_cb(const actionlib::SimpleClientGoalState& state,
                                  const saisun_msgs::InitialResultConstPtr& result)
{
    ROS_INFO("I'm in initial ac callback, success is %d", result->success);
}

void SaisunWrapper::action_start(void)
{
    switch (receive_type_)
    {
    case receiveMessageTypes::GET_INIT:
    {
        saisun_msgs::InitialActionGoal init_goal;
        actionlib::SimpleClientGoalState::StateEnum server_state;
        server_state = saisun_init_ac_->getState().state_;
        ROS_INFO("RETUREN STATUS IS %d, ACTION CONNECT %d",server_state,saisun_init_ac_->isServerConnected());
        if (!saisun_init_ac_->isServerConnected())
        {
            if (server_state!= actionlib::SimpleClientGoalState::LOST)
            {
                saisun_init_ac_->stopTrackingGoal();
            }
            
            ROS_INFO("not connect");
            break;
        }
        
        if (server_state != actionlib::SimpleClientGoalState::PENDING)
        {
            saisun_init_ac_->sendGoal(init_goal.goal,bind(&SaisunWrapper::initial_ac_cb,this,_1,_2));
            /* code */
        }
        
        ROS_INFO("in init");
        break;
    }
    case receiveMessageTypes::GET_TRIG:
    {
        // saisun_msgs::
        ROS_INFO("in trig");
        break;
    }
    case receiveMessageTypes::GET_DATA:
    {
        ROS_INFO("in data");
        break;
    }    
    default:
        break;
    }
}

void SaisunWrapper::control_loop(void)
{
    uint8_t msg_body[8];
    saisun_init_ac_->waitForServer();
    // saisun_trig_ac_->waitForServer();
    // saisun_result_ac_->waitForServer();
    // TODO: ADD set vision state public funciton in state cpp
    while (ros::ok())
    {
        bzero(msg_body,8);
        // robot state
        parse_robot_pos();
        // cmd
        if(saisunState_->get_robot_cmd(receive_type_,msg_body))
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
    std::cout << "I'm in ending" << std::endl;
    saisun_wrapper.halt();

    return 0;    
}