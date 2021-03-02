#include "saisun_hw/ros_wrapper.h"

SaisunWrapper::SaisunWrapper():Node("saisun_driver")
{
    ros_init();
    init();
}

void SaisunWrapper::ros_init(void)
{
    // param get 
    this->declare_parameter("saisun_ip","172.16.1.30"); // Default value of 0
    this->declare_parameter("saisun_port",9999);  // Default string is given
    this->declare_parameter("use_net_sequence",false);
    this->declare_parameter("algorithm_version","0.1");

    if (!this->get_parameter("saisun_ip",host_))
    {   RCLCPP_ERROR(this->get_logger(),"No saisun_ip param");}
    int port;
    if (!this->get_parameter("saisun_port", port))
    {   RCLCPP_ERROR(this->get_logger(),"No saisun_port param");}
    port_ = port;
    if (!this->get_parameter("use_net_sequence", use_net_sequence_))
    {   RCLCPP_ERROR(this->get_logger(),"No use_net_sequence param");}
    RCLCPP_INFO(this->get_logger(),"use_net_sequence_ is %d",use_net_sequence_);
    if (!this->get_parameter("algorithm_version", algorithm_version_))
    {   RCLCPP_ERROR(this->get_logger(),"No algorithm_version param");}

    robot_pose_pub_ = this->create_publisher<saisun_msgs::msg::RobotPose>("/saisun_pose",10);

    saisun_init_ac_ = rclcpp_action::create_client<SaisunInitial>(
        this,"saisun_robot_initial");
    saisun_trig_ac_ = rclcpp_action::create_client<SaisunTrigger>(
        this,"saisun_robot_trigger");
    saisun_result_ac_ = rclcpp_action::create_client<SaisunResult>(
        this,"saisun_robot_get_result");

    robot_pose_init_ = false;
    init_goal_done_ = true;
    trig_goal_done_ = true;
    result_goal_done_ = true;
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
        std::bind(&SaisunWrapper::control_loop,this)));
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
    robot_pose_pub_->publish(robot_pos_);
}

void SaisunWrapper::initial_ac_cb(const GoalHandleInitial::WrappedResult& result)
{
    const uint32_t body_len = 2;
    uint8_t msg_body[body_len];
    memset(msg_body,0,body_len);

    init_goal_done_ = true;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Initial Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Initial Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Initial Goal has an unknown result code");
        return;
    }
    
    if (result.result->success)
    {
        saisunState_->set_vision_state(visionStateTypes::VISION_OK);
        saisunState_->pack(sendMessageTypes::SEND_INIT,msg_body,body_len);
        RCLCPP_INFO(this->get_logger(),"Vision initial successed");
    }else{
        saisunState_->set_vision_state(visionStateTypes::OTHER_ERROR);
        saisunState_->pack(sendMessageTypes::SEND_ERROR,msg_body,body_len);
    }
}


void SaisunWrapper::trigger_ac_cb(const GoalHandleTrigger::WrappedResult& result)
{
    const uint32_t body_len = 2;
    uint8_t msg_body[body_len];
    memset(msg_body,0,body_len);

    trig_goal_done_ = true;    

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Trigger Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Trigger Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Trigger Goal has an unknown result code");
        return;
    }

    msg_body[0] = result.result->detection_state;
    msg_body[1] = result.result->return_scene_group;
    saisunState_->pack(sendMessageTypes::SEND_TRIG,msg_body,body_len);
}

void SaisunWrapper::result_ac_cb(const GoalHandleResult::WrappedResult& result)
{
    result_goal_done_ = true;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Result Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Result Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Result Goal has an unknown result code");
        return;
    }

    uint8_t object_num = result.result->object_num;
    std::vector<saisun_msgs::msg::ObjectPose> object_pose;
    object_pose.resize(object_num);
    object_pose = result.result->object_pos;

    const uint32_t body_len = 4 + 64 * object_num;
    const size_t max_body_len = 4 + 64 * 8;
    uint8_t msg_body[max_body_len];
    memset(msg_body,0,max_body_len);
    
    msg_body[0] = result.result->detection_state;
    msg_body[1] = result.result->return_scene_group;
    msg_body[2] = object_num;
    // RCLCPP_INFO(this->get_logger(),"Result Goal was successed %d", msg_body[0]);
    size_t offset = 4;

    for (size_t num = 0; num < object_pose.size(); num++)
    {
        // linear
        memcpy(&msg_body[offset + num * 64 + 0*4],&object_pose[num].o_linear.x,sizeof(float)); 
        memcpy(&msg_body[offset + num * 64 + 1*4],&object_pose[num].o_linear.y,sizeof(float)); 
        memcpy(&msg_body[offset + num * 64 + 2*4],&object_pose[num].o_linear.z,sizeof(float)); 
        // angular
        memcpy(&msg_body[offset + num * 64 + 3*4],&object_pose[num].o_angular.x,sizeof(float));
        memcpy(&msg_body[offset + num * 64 + 4*4],&object_pose[num].o_angular.y,sizeof(float));
        memcpy(&msg_body[offset + num * 64 + 5*4],&object_pose[num].o_angular.z,sizeof(float));
        RCLCPP_INFO(this->get_logger(),"Result object position is[%f, %f, %f, %f, %f, %f]",object_pose[num].o_linear.x,object_pose[num].o_linear.y,object_pose[num].o_linear.z,object_pose[num].o_angular.x,object_pose[num].o_angular.y,object_pose[num].o_angular.z);
        // vision angular
        memcpy(&msg_body[offset + num * 64 + 6*4],&object_pose[num].v_angular.x,sizeof(float));
        memcpy(&msg_body[offset + num * 64 + 7*4],&object_pose[num].v_angular.y,sizeof(float));
        memcpy(&msg_body[offset + num * 64 + 8*4],&object_pose[num].v_angular.z,sizeof(float));
        // position filed
        memcpy(&msg_body[offset + num * 64 + 9*4],&object_pose[num].p_filed,sizeof(float));
    }
    saisunState_->pack(sendMessageTypes::SEND_DATA,msg_body,body_len);
}

void SaisunWrapper::initial_response(std::shared_future<GoalHandleInitial::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Initial Goal was rejected by server");
    }
}

void SaisunWrapper::trigger_response(std::shared_future<GoalHandleTrigger::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Trigger Goal was rejected by server");
    }
}

void SaisunWrapper::result_response(std::shared_future<GoalHandleResult::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Result Goal was rejected by server");
    }
}

void SaisunWrapper::wait_vision_node(void)
{
    RCLCPP_INFO(this->get_logger(),"Waiting for vision node ....");
    saisun_init_ac_->wait_for_action_server();
    saisun_trig_ac_->wait_for_action_server();
    saisun_result_ac_->wait_for_action_server();
    saisunState_->set_vision_state(visionStateTypes::VISION_OK);
    RCLCPP_INFO(this->get_logger(),"Vision node is connected. Start the TCP protocol with Saisun Robot");
}

void SaisunWrapper::action_start(receiveMessageTypes cmd, uint8_t *msg)
{
    using namespace std::placeholders;

    receiveMessageTypes receive_type = cmd;
    uint8_t msg_body[8];
    memcpy(msg_body,msg,8);
    switch (receive_type)
    {
    case receiveMessageTypes::GET_INIT:
    {
        auto init_goal = SaisunInitial::Goal();
        auto send_goal_option = rclcpp_action::Client<SaisunInitial>::SendGoalOptions();
        send_goal_option.goal_response_callback = 
            std::bind(&SaisunWrapper::initial_response, this, _1);
        send_goal_option.result_callback=
            std::bind(&SaisunWrapper::initial_ac_cb,this,_1);
        if (!saisun_init_ac_->action_server_is_ready())
        {
            init_goal_done_ = true;
        }
        if(init_goal_done_)
        {
            saisun_init_ac_->async_send_goal(init_goal,send_goal_option);
            RCLCPP_INFO(this->get_logger(),"in init req");
            init_goal_done_ = false;
        }
        break;
    }
    case receiveMessageTypes::GET_TRIG:
    {
        auto trig_goal = saisun_msgs::action::Trigger::Goal();
        auto send_goal_option = rclcpp_action::Client<SaisunTrigger>::SendGoalOptions();
        send_goal_option.goal_response_callback = 
            std::bind(&SaisunWrapper::trigger_response, this, _1);
        send_goal_option.result_callback=
            std::bind(&SaisunWrapper::trigger_ac_cb,this,_1);
        if (!saisun_trig_ac_->action_server_is_ready())
        {
            trig_goal_done_ = true;
        }
        if (trig_goal_done_)
        { 
            trig_goal.trigger = msg_body[0];
            trig_goal.scene_group = msg_body[1];
            memcpy(&trig_goal.scene,&msg_body[4],sizeof(uint16_t));
            saisun_trig_ac_->async_send_goal(trig_goal,send_goal_option);
            RCLCPP_INFO(this->get_logger(),"in trig req");
            trig_goal_done_ = false;
        }

        break;
    }
    case receiveMessageTypes::GET_DATA:
    {
        auto result_goal = saisun_msgs::action::GetResult::Goal();
        auto send_goal_option = rclcpp_action::Client<SaisunResult>::SendGoalOptions();
        send_goal_option.goal_response_callback = 
            std::bind(&SaisunWrapper::result_response, this, _1);
        send_goal_option.result_callback=
            std::bind(&SaisunWrapper::result_ac_cb,this,_1);
        if (!saisun_result_ac_->action_server_is_ready())
        {
            result_goal_done_ = true;
        }
        if(result_goal_done_)
        {
            result_goal.scene_group = msg_body[1];
            saisun_result_ac_->async_send_goal(result_goal,send_goal_option);
            RCLCPP_INFO(this->get_logger(),"in result req");
            result_goal_done_ = false;
        }
        break;
    }    
    default:
        break;
    }
}

void SaisunWrapper::control_loop(void)
{
    wait_vision_node();
    while (rclcpp::ok())
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

int main(int argc, char *argv[])
{
    // ros init
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor executors;
    auto saisun_wrapper = std::make_shared<SaisunWrapper>();
    // spin multiple threads
    executors.add_node(saisun_wrapper);
    saisun_wrapper->start();
    executors.spin();
    // end this node
    rclcpp::shutdown();
    saisun_wrapper->halt();
    printf("I'm in end");
    return 0;
}
