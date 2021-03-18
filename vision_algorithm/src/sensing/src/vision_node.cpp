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
#include "saisun_msgs/srv/vision_algorithm.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"

#include "example_interfaces/srv/set_bool.hpp"

#include <stdlib.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>

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

    rclcpp::Client<saisun_msgs::srv::VisionAlgorithm>::SharedPtr vision_client_;

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
    cv::Mat color_mat_;
    PointCloudColor::Ptr cloud_ptr_;
    geometry_msgs::msg::Pose pose_;
    Eigen::Vector3f pose_euler_;

    // rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr test_srv_;
    bool detect_result_ = false;

    void init(void);
    void ros_init(void);

    // void test_handle(
    //     const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    //     const std::shared_ptr<example_interfaces::srv::SetBool::Response> response);
};

VisionNode::VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("vision_algorithm",options)
{
    init();
    ros_init();
}

void VisionNode::init(void)
{
    // roi_.x_offset = -800.0;
    // roi_.y_offset = -1404.0;
    // roi_.z_offset = 600;
    // roi_.width = 2216.0;
    // roi_.height = 2274.0;
    // roi_.depth = 800.0;
    roi_.x_offset = -800;
    roi_.y_offset = -800;
    roi_.z_offset = 1500;
    roi_.width = 1600;
    roi_.height = 1600;
    roi_.depth = 1670;

    cloud_ptr_.reset(new PointCloudColor());
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

    this->vision_client_ = this->create_client<saisun_msgs::srv::VisionAlgorithm>("object_pose_srv");

    // this->test_srv_ = this->create_service<example_interfaces::srv::SetBool>("test_service",std::bind(&VisionNode::test_handle,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
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
    // sensor_msgs::msg::Image image_msgs;
    // sensor_msgs::msg::PointCloud2 cloud_msgs;
    // auto vision_srv = std::make_shared<saisun_msgs::srv::VisionAlgorithm::Request>();
    // std::shared_future<std::shared_ptr<saisun_msgs::srv::VisionAlgorithm_Response>> vision_res;
    using namespace std::chrono_literals;

    double width = 0.0;
    double height = 0.0;
    
    detect_result_ = sensing_server_.senseObjectPose(pose_,width, height);

    if (!detect_result_)
    {
        as_result->detection_state = 0x01;
        goal_handle->succeed(as_result);
        return;
    }
    
    // cv_bridge::CvImage img_bridge;
    // std_msgs::msg::Header header;
    // header.stamp = rclcpp::Node::get_clock()->now();
    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, color_mat_);
    // img_bridge.toImageMsg(image_msgs);

    // RCLCPP_INFO(this->get_logger(),"CONVERT IMAGE FINISH");

    // pcl::toROSMsg(*cloud_ptr_,cloud_msgs);

    // RCLCPP_INFO(this->get_logger(),"CONVERT PCL FINISH");

    // vision_srv->img = image_msgs;
    // vision_srv->pcl = cloud_msgs;
    
    // std::cout << "size of converted" <<vision_srv->pcl.data.size() << std::endl;
    // std::cout << "size of before" << cloud_ptr_->points.size() << std::endl;

    // vision_res = vision_client_->async_send_request(vision_srv);

    // rclcpp::FutureReturnCode srv_response = rclcpp::spin_until_future_complete(this->shared_from_this(),vision_res,10s);
    

    // if (vision_res.get()->success && srv_response == rclcpp::FutureReturnCode::SUCCESS)
    // if (vision_res.get()->success)
    {
        // pose_ = vision_res.get()->object;

        as_result->detection_state = 0x00;
        as_result->return_scene_group = goal->scene_group;
        Eigen::Quaternionf quart(pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w);
        pose_euler_ = quart.matrix().eulerAngles(2,1,0);
    }
    // else{
    //     as_result->detection_state = 0x01;
    //     RCLCPP_ERROR(this->get_logger(),"Failed to call service vision_algorithm");
    // }
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
    rclcpp::executors::MultiThreadedExecutor executors;
    auto action_server = std::make_shared<VisionNode>();

    executors.add_node(action_server);
    executors.spin();

    rclcpp::shutdown();

    return 0;
}
