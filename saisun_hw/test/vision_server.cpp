#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "saisun_msgs/InitialAction.h"

class visionServer
{
public:
    visionServer(ros::NodeHandle &nh,std::string name):
                nh_(nh),
                as_(nh_,name
                ,boost::bind(&visionServer::initial_execute_cb,this,_1)
                ,false),
                action_name_(name)
    {
        as_.start();
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<saisun_msgs::InitialAction> as_;
    std::string action_name_;


    // create action result
    saisun_msgs::InitialResult result_;

    void ros_init();
    void initial_execute_cb(const saisun_msgs::InitialGoalConstPtr & goal);
};


void visionServer::initial_execute_cb(const saisun_msgs::InitialGoalConstPtr & goal)
{
    ROS_INFO("I'm in action call begin");
    // If init success, set result = true otherwise false.
    result_.success = true;
    ros::Duration(2).sleep();
    ROS_INFO("I'm in action call end");
    as_.setSucceeded(result_);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "vision_server");
    ros::NodeHandle nh;

    visionServer vs_(nh,"saisun_robot/saisun_robot_initial");
    
    ros::spin();
}