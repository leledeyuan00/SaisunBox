#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "saisun_msgs/InitialAction.h"
#include "saisun_msgs/TriggerAction.h"
#include "saisun_msgs/GetResultAction.h"
#include "saisun_msgs/ObjectPose.h"
#include "saisun_msgs/RobotPose.h"
#include "saisun_msgs/Vector3f.h"

class visionServer
{
public:
    visionServer(ros::NodeHandle &nh);
    void start(void);

private:
    ros::NodeHandle nh_;
    std::shared_ptr<actionlib::SimpleActionServer<saisun_msgs::InitialAction>> init_as_;
    std::shared_ptr<actionlib::SimpleActionServer<saisun_msgs::TriggerAction>> trig_as_;
    std::shared_ptr<actionlib::SimpleActionServer<saisun_msgs::GetResultAction>> result_as_;

    ros::Subscriber saisun_sub_;
    
    saisun_msgs::RobotPose robot_pose_;

    void ros_init(void);
    void saisunRobotCallback(const saisun_msgs::RobotPoseConstPtr &msg);
    void initial_execute_cb(const saisun_msgs::InitialGoalConstPtr& goal);
    void triger_execute_cb(const saisun_msgs::TriggerGoalConstPtr& goal);
    void result_execute_cb(const saisun_msgs::GetResultGoalConstPtr& goal);
};

visionServer::visionServer(ros::NodeHandle &nh):
                nh_(nh)
{
    ros_init();
}

void visionServer::ros_init(void)
{
    saisun_sub_ = nh_.subscribe<saisun_msgs::RobotPose>("/saisun_pose", 10, &visionServer::saisunRobotCallback,this);
    
    init_as_.reset(new actionlib::SimpleActionServer<saisun_msgs::InitialAction>(
        nh_,"/saisun_robot/saisun_robot_initial",boost::bind(&visionServer::initial_execute_cb,this,_1),false));

    trig_as_.reset(new actionlib::SimpleActionServer<saisun_msgs::TriggerAction>(
        nh_,"/saisun_robot/saisun_robot_trigger",boost::bind(&visionServer::triger_execute_cb,this,_1),false));
    
    result_as_.reset(new actionlib::SimpleActionServer<saisun_msgs::GetResultAction>(
        nh_,"/saisun_robot/saisun_robot_get_result",boost::bind(&visionServer::result_execute_cb,this,_1),false));    
}

void visionServer::start(void)
{
    init_as_->start();
    trig_as_->start();
    result_as_->start();
}

void visionServer::saisunRobotCallback(const saisun_msgs::RobotPoseConstPtr &msg)
{
    robot_pose_ = *msg;
}
    
void visionServer::initial_execute_cb(const saisun_msgs::InitialGoalConstPtr & goal)
{
    // create action result
    saisun_msgs::InitialResult result;
    ROS_INFO("I'm in action call begin");
    // If init success, set result = true otherwise false.
    result.success = true;
    
    ROS_INFO("I'm in action call end");
    init_as_->setSucceeded(result);
}

void visionServer::triger_execute_cb(const saisun_msgs::TriggerGoalConstPtr& goal)
{
    /* get command */
    // 0 -> measure and return state
    // 1 -> just return state
    uint8_t trigger = goal->trigger;
    // method
    uint8_t scene_group = goal->scene_group;
    // vision serial
    uint16_t vision_scene = goal->scene;

    /* return result */
    saisun_msgs::TriggerResult result;
    result.detection_state;
    result.return_scene_group;
    trig_as_->setSucceeded(result);
}

void visionServer::result_execute_cb(const saisun_msgs::GetResultGoalConstPtr& goal)
{
    /* get command */
    // method
    uint8_t scene_group = goal->scene_group;

    /* return result */
    // example return
    uint8_t object_num = 1; // number <= 8
    std::vector<saisun_msgs::ObjectPose> object_pose;
    saisun_msgs::ObjectPose temp_pose;
    temp_pose.o_angular.x = 30;
    temp_pose.o_angular.y = 0;
    temp_pose.o_angular.z = 0;

    temp_pose.o_linear.x = 0.1;
    temp_pose.o_linear.y = 0.2;
    temp_pose.o_linear.z = 0.1;

    temp_pose.v_angular.x = 10;
    temp_pose.v_angular.y = 20;
    temp_pose.v_angular.z = 30;

    object_pose.push_back(temp_pose);
    

    saisun_msgs::GetResultResult result;
    result.detection_state = 0x00;
    result.object_num = object_num;
    result.object_pos = object_pose;
    result.return_scene_group;
    result_as_->setSucceeded(result);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vision_server");
    ros::NodeHandle nh;

    visionServer vs_(nh);
    vs_.start();
    
    ros::spin();
}