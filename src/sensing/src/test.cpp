#include "sensing/SensingServer.hpp"
#include "Eigen/Eigen"
#include "sensing/visibility_control.h"

#include <functional>
#include <memory>
#include <thread>
#include <inttypes.h>


#include "saisun_msgs/action/initial.hpp"
#include "saisun_msgs/action/trigger.hpp"
#include "saisun_msgs/action/get_result.hpp"
#include "saisun_msgs/msg/object_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class VisionNode: public rclcpp::Node
{
public:
    explicit VisionNode(const rclcpp::NodeOptions & options);
private:
    using SaisunInitial = saisun_msgs::action::Initial;
    using SaisunTrigger = saisun_msgs::action::Trigger;
    using SaisunResult  = saisun_msgs::action::GetResult;

    using GoalHandleInitial = rclcpp_action::ServerGoalHandle<SaisunInitial>;
    using GoalHandleTrigger = rclcpp_action::ServerGoalHandle<SaisunTrigger>;
    using GoalHandleResult  = rclcpp_action::ServerGoalHandle<SaisunResult>;

    rclcpp_action::Server<SaisunInitial>::SharedPtr initial_as_;
    rclcpp_action::Server<SaisunTrigger>::SharedPtr trigger_as_;
    rclcpp_action::Server<SaisunResult>::SharedPtr  result_as_;

    rclcpp_action::GoalResponse initial_res(const rclcpp_action::GoalUUID & uuid,
                                           std::shared_ptr<const SaisunInitial::Goal> goal);
    rclcpp_action::GoalResponse trigger_res(const rclcpp_action::GoalUUID & uuid,
                                           std::shared_ptr<const SaisunTrigger::Goal> goal);
    rclcpp_action::GoalResponse result_res(const rclcpp_action::GoalUUID & uuid,
                                           std::shared_ptr<const SaisunResult::Goal> goal);

    rclcpp_action::CancelResponse initial_cancel(const std::shared_ptr<GoalHandleInitial> goal_handle);
    rclcpp_action::CancelResponse trigger_cancel(const std::shared_ptr<GoalHandleTrigger> goal_handle);
    rclcpp_action::CancelResponse result_cancel(const std::shared_ptr<GoalHandleResult> goal_handle);

    void initial_accepted(const std::shared_ptr<GoalHandleInitial> goal_handle);
    void trigger_accepted(const std::shared_ptr<GoalHandleTrigger> goal_handle);
    void result_accepted(const std::shared_ptr<GoalHandleResult> goal_handle);

    SensingServer sensing_server_;
    RegionOfInterest roi_;
    geometry_msgs::msg::Pose pose_;
    Eigen::Vector3f pose_euler_;
    bool detect_result_ = false;

    void init(void);
    void ros_init(void);
};

VisionNode::VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("vision_algorithm",options)
{
    init();
    ros_init();
}

void VisionNode::init(void)
{
    roi_.x_offset = -800.0;
    roi_.y_offset = -1404.0;
    roi_.z_offset = 600;
    roi_.width = 2216.0;
    roi_.height = 2274.0;
    roi_.depth = 800.0;

    sensing_server_.config(CAMERALMODEL::SMARTEYE_HV1000, roi_);
}

void VisionNode::ros_init(void)
{
    this->initial_as_ = rclcpp_action::create_server<SaisunInitial>(
        this,
        "saisun_robot_initial",
        std::bind(&VisionNode::initial_res,this,std::placeholders::_1,std::placeholders::_2),
        std::bind(&VisionNode::initial_cancel,this,std::placeholders::_1),
        std::bind(&VisionNode::initial_accepted,this,std::placeholders::_1));
    
    this->trigger_as_ = rclcpp_action::create_server<SaisunTrigger>(
        this,
        "saisun_robot_trigger",
        std::bind(&VisionNode::trigger_res,this,std::placeholders::_1,std::placeholders::_2),
        std::bind(&VisionNode::trigger_cancel,this,std::placeholders::_1),
        std::bind(&VisionNode::trigger_accepted,this,std::placeholders::_1));

    this->result_as_ = rclcpp_action::create_server<SaisunResult>(
        this,
        "saisun_robot_get_result",
        std::bind(&VisionNode::result_res,this,std::placeholders::_1,std::placeholders::_2),
        std::bind(&VisionNode::result_cancel,this,std::placeholders::_1),
        std::bind(&VisionNode::result_accepted,this,std::placeholders::_1));
}

// Action Initial function
rclcpp_action::GoalResponse VisionNode::initial_res(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SaisunInitial::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received Initial request");
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse VisionNode::trigger_res(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SaisunTrigger::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received Trigger request");
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse VisionNode::result_res(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SaisunResult::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received Result request");
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// Action Cancel Function
rclcpp_action::CancelResponse VisionNode::initial_cancel(const std::shared_ptr<GoalHandleInitial> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse VisionNode::trigger_cancel(const std::shared_ptr<GoalHandleTrigger> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse VisionNode::result_cancel(const std::shared_ptr<GoalHandleResult> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
// Action Result Function 
void VisionNode::initial_accepted(const std::shared_ptr<GoalHandleInitial> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto as_result = std::make_shared<SaisunInitial::Result>();
    as_result->success = true;
    goal_handle->succeed(as_result);
}

void VisionNode::trigger_accepted(const std::shared_ptr<GoalHandleTrigger> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto as_result = std::make_shared<SaisunTrigger::Result>();
    
    double width = 0.0;
    double height = 0.0;
    
    detect_result_ = sensing_server_.senseObjectPose(pose_, width, height);
    if (detect_result_)
    {
        as_result->detection_state = 0x00;
        as_result->return_scene_group = goal->scene_group;
        Eigen::Quaternionf quart(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
        pose_euler_ = quart.matrix().eulerAngles(2,1,0);
    }
    else{
        as_result->detection_state = 0x01;
    }
    goal_handle->succeed(as_result);
}

void VisionNode::result_accepted(const std::shared_ptr<GoalHandleResult> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto as_result = std::make_shared<SaisunResult::Result>();
    
    if(detect_result_) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensing succeed!");
        as_result->detection_state = 0x00;
        as_result->object_num = 1;

        saisun_msgs::msg::ObjectPose obPose;
        obPose.o_linear.x = pose_.position.x;
        obPose.o_linear.y = pose_.position.y;
        obPose.o_linear.z = pose_.position.z;
        obPose.o_angular.x = pose_euler_[0];
        obPose.o_angular.y = pose_euler_[1];
        obPose.o_angular.z = pose_euler_[2];
        as_result->object_pos.push_back(obPose);
        as_result->return_scene_group = goal->scene_group;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Sensing failed!");
        as_result->return_scene_group = goal->scene_group;
        as_result->detection_state = 0x01;
    }
    goal_handle->succeed(as_result);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    auto action_server = std::make_shared<VisionNode>();

    rclcpp::spin(action_server);

    rclcpp::shutdown();

    return 0;
}
