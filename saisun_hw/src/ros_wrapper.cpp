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

    robot_pose_pub_ = nh_.advertise<saisun_msgs::RobotPose>("/saisun_pose", 10);
    
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
}

void SaisunWrapper::start(void)
{
    control_loop_thread_.reset(new std::thread(
        boost::bind(&SaisunWrapper::control_loop,this)));
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

    robot_pos_.linear.x  = robot_pos[0];
    robot_pos_.linear.y  = robot_pos[1];
    robot_pos_.linear.z  = robot_pos[2];
    robot_pos_.angular.x = robot_pos[3];
    robot_pos_.angular.y = robot_pos[4];
    robot_pos_.angular.z = robot_pos[5];

}

void SaisunWrapper::publish_msgs(void)
{
    robot_pose_pub_.publish(robot_pos_);
}

// action callback
void SaisunWrapper::initial_ac_cb(const actionlib::SimpleClientGoalState& state,
                                  const saisun_msgs::InitialResultConstPtr& result)
{
    uint32_t body_len = 2;
    uint8_t msg_body[body_len];
    memset(msg_body,0,body_len);
    if (result->success)
    {
        saisunState_->set_vision_state(visionStateTypes::VISION_OK);
        saisunState_->pack(sendMessageTypes::SEND_INIT,msg_body,body_len);
        ROS_INFO("Vision initial successed");
    }else{
        saisunState_->set_vision_state(visionStateTypes::OTHER_ERROR);
        saisunState_->pack(sendMessageTypes::SEND_ERROR,msg_body,body_len);
    }
    
}

void SaisunWrapper::trigger_ac_cb(const actionlib::SimpleClientGoalState& state,
                                  const saisun_msgs::TriggerResultConstPtr& result)
{
    uint32_t body_len = 2;
    uint8_t msg_body[body_len];
    memset(msg_body,0,body_len);
    msg_body[0] = result->detection_state;
    msg_body[1] = result->return_scene_group;
    saisunState_->pack(sendMessageTypes::SEND_TRIG,msg_body,body_len);
}

void SaisunWrapper::result_ac_cb(const actionlib::SimpleClientGoalState& state,
                                 const saisun_msgs::GetResultResultConstPtr& result)
{
    uint8_t object_num = result->object_num;
    std::vector<saisun_msgs::ObjectPose> object_pose;
    object_pose.resize(object_num);
    object_pose = result->object_pos;

    uint32_t body_len = 4 + 64 * object_num;
    uint8_t msg_body[body_len];
    memset(msg_body,0,body_len);
    
    msg_body[0] = result->detection_state;
    msg_body[1] = result->return_scene_group;
    msg_body[2] = object_num;

    for (size_t num = 0; num < object_pose.size(); num++)
    {
        // linear
        memcpy(&msg_body[num * 64 + 0*4],&object_pose[num].o_linear.x,sizeof(float)); 
        memcpy(&msg_body[num * 64 + 1*4],&object_pose[num].o_linear.y,sizeof(float)); 
        memcpy(&msg_body[num * 64 + 2*4],&object_pose[num].o_linear.z,sizeof(float)); 
        // angular
        memcpy(&msg_body[num * 64 + 3*4],&object_pose[num].o_angular.x,sizeof(float));
        memcpy(&msg_body[num * 64 + 4*4],&object_pose[num].o_angular.y,sizeof(float));
        memcpy(&msg_body[num * 64 + 5*4],&object_pose[num].o_angular.z,sizeof(float));
        // vision angular
        memcpy(&msg_body[num * 64 + 6*4],&object_pose[num].v_angular.x,sizeof(float));
        memcpy(&msg_body[num * 64 + 7*4],&object_pose[num].v_angular.y,sizeof(float));
        memcpy(&msg_body[num * 64 + 8*4],&object_pose[num].v_angular.z,sizeof(float));
        // position filed
        memcpy(&msg_body[num * 64 + 9*4],&object_pose[num].p_filed,sizeof(float));
    }
    saisunState_->pack(sendMessageTypes::SEND_DATA,msg_body,body_len);
}

void SaisunWrapper::action_start(receiveMessageTypes cmd, uint8_t *msg)
{
    receiveMessageTypes receive_type = cmd;
    uint8_t msg_body[8];
    memcpy(msg_body,msg,8);
    switch (receive_type)
    {
    case receiveMessageTypes::GET_INIT:
    {
        saisun_msgs::InitialGoal init_goal;
        actionlib::SimpleClientGoalState::StateEnum server_state;
        server_state = saisun_init_ac_->getState().state_;
        if (!saisun_init_ac_->isServerConnected())
        {
            if (server_state!= actionlib::SimpleClientGoalState::LOST)
            {
                saisun_init_ac_->stopTrackingGoal();
            }
            ROS_ERROR("Vision system <Initial Action> not connect");
            break;
        }
        if (server_state != actionlib::SimpleClientGoalState::PENDING)
        {
            saisun_init_ac_->sendGoal(init_goal,bind(&SaisunWrapper::initial_ac_cb,this,_1,_2));
        }
        
        ROS_INFO("in init");
        break;
    }
    case receiveMessageTypes::GET_TRIG:
    {
        saisun_msgs::TriggerGoal trig_goal;
        actionlib::SimpleClientGoalState::StateEnum server_state;
        server_state = saisun_trig_ac_->getState().state_;
        if (!saisun_trig_ac_->isServerConnected())
        {
            if (server_state!= actionlib::SimpleClientGoalState::LOST)
            {
                saisun_trig_ac_->stopTrackingGoal();
            }
            ROS_ERROR("Vision system <Trigger Action> not connect");
            break;
        }
        if (server_state != actionlib::SimpleClientGoalState::PENDING)
        {
            trig_goal.trigger = msg_body[0];
            trig_goal.scene_group = msg_body[1];
            memcpy(&trig_goal.scene,&msg_body[4],sizeof(uint16_t));
            saisun_trig_ac_->sendGoal(trig_goal,bind(&SaisunWrapper::trigger_ac_cb,this,_1,_2));
        }
        ROS_INFO("in trig");
        break;
    }
    case receiveMessageTypes::GET_DATA:
    {
        saisun_msgs::GetResultGoal result_goal;
        actionlib::SimpleClientGoalState::StateEnum server_state;
        server_state = saisun_result_ac_->getState().state_;
        if (!saisun_result_ac_->isServerConnected())
        {
            if (server_state!= actionlib::SimpleClientGoalState::LOST)
            {
                saisun_result_ac_->stopTrackingGoal();
            }
            ROS_ERROR("Vision system <Result Action> not connect");
            break;
        }
        if (server_state != actionlib::SimpleClientGoalState::PENDING)
        {
            result_goal.scene_group = msg_body[1];
            saisun_result_ac_->sendGoal(result_goal,bind(&SaisunWrapper::result_ac_cb,this,_1,_2));
        }
        break;
    }    
    default:
        break;
    }
}

void SaisunWrapper::wait_vision_node(void)
{
    ROS_INFO("Waiting for vision node ....");
    saisun_init_ac_->waitForServer();
    saisun_trig_ac_->waitForServer();
    saisun_result_ac_->waitForServer();
    saisunState_->set_vision_state(visionStateTypes::VISION_OK);
    ROS_INFO("Vision node is connected. Start the TCP protocol with Saisun Robot");
}

void SaisunWrapper::control_loop(void)
{
    wait_vision_node();
    while (ros::ok())
    {
        /* polling */
        bzero(receive_msg_body_,8);
        // robot state
        parse_robot_pos();
        // cmd
        if(saisunState_->get_robot_cmd(receive_type_,receive_msg_body_))
        {
            action_start(receive_type_,receive_msg_body_);
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