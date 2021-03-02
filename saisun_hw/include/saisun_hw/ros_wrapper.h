#ifndef ROS_WRAPPER_H_
#define ROS_WRAPPER_H_

#include <functional>
#include <memory>
#include <thread>
#include <future>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "saisun_msgs/msg/vector3f.hpp"
#include "saisun_msgs/action/trigger.hpp"
#include "saisun_msgs/action/initial.hpp"
#include "saisun_msgs/action/get_result.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "saisun_msgs/msg/object_pose.hpp"
#include "saisun_msgs/msg/robot_pose.hpp"
#include "saisun_state.h"

class SaisunWrapper : public rclcpp::Node
{
public:
    SaisunWrapper();

    void start(void);
    void halt(void);

private:
    using SaisunInitial = saisun_msgs::action::Initial;
    using SaisunTrigger = saisun_msgs::action::Trigger;
    using SaisunResult  = saisun_msgs::action::GetResult;

    using GoalHandleInitial = rclcpp_action::ClientGoalHandle<SaisunInitial>;
    using GoalHandleTrigger = rclcpp_action::ClientGoalHandle<SaisunTrigger>;
    using GoalHandleResult  = rclcpp_action::ClientGoalHandle<SaisunResult>;

    std::shared_ptr<SaisunState> saisunState_;
    std::shared_ptr<SaisunCom> saisunCom_;
    receiveMessageTypes receive_type_;
    sendMessageTypes send_type_;
    uint8_t receive_msg_body_[8];

    rclcpp::Publisher<saisun_msgs::msg::RobotPose>::SharedPtr robot_pose_pub_;
    rclcpp_action::Client<SaisunInitial>::SharedPtr saisun_init_ac_;
    rclcpp_action::Client<SaisunTrigger>::SharedPtr   saisun_trig_ac_;
    rclcpp_action::Client<SaisunResult>::SharedPtr saisun_result_ac_;

    bool init_goal_done_;
    bool trig_goal_done_;
    bool result_goal_done_;

    bool robot_pose_init_;

    std::string host_;
    unsigned int port_;
    std::string algorithm_version_;
    bool use_net_sequence_;

    saisun_msgs::msg::RobotPose robot_pos_; 

    std::shared_ptr<std::thread> control_loop_thread_;

    void ros_init(void);
    void init(void);

    void action_start(receiveMessageTypes cmd, uint8_t *msg);
    
    void initial_response(std::shared_future<GoalHandleInitial::SharedPtr> future);
    void trigger_response(std::shared_future<GoalHandleTrigger::SharedPtr> future);
    void result_response(std::shared_future<GoalHandleResult::SharedPtr> future);

    void initial_ac_cb(const GoalHandleInitial::WrappedResult& result);
    void trigger_ac_cb(const GoalHandleTrigger::WrappedResult& result);
    void result_ac_cb(const GoalHandleResult::WrappedResult& result);

    void parse_robot_pos(void);
    void publish_msgs(void);
    void control_loop(void);

    void wait_vision_node(void);
};

#endif