#ifndef ROS_WRAPPER_H_
#define ROS_WRAPPER_H_

#include "ros/ros.h"
#include "saisun_state.h"
#include "actionlib/client/simple_action_client.h"
#include "saisun_msgs/InitialAction.h"
#include "saisun_msgs/TriggerAction.h"
#include "saisun_msgs/GetResultAction.h"
#include "saisun_msgs/RobotPose.h"
#include "saisun_msgs/ObjectPose.h"


class SaisunWrapper
{
public:
    SaisunWrapper(ros::NodeHandle &nh);

    void start(void);
    void halt(void);


private:
    std::shared_ptr<SaisunState> saisunState_;
    std::shared_ptr<SaisunCom> saisunCom_;
    receiveMessageTypes receive_type_;
    sendMessageTypes send_type_;
    uint8_t receive_msg_body_[8];


    ros::NodeHandle nh_;
    ros::Publisher robot_pose_pub_;
    std::shared_ptr<actionlib::SimpleActionClient<saisun_msgs::InitialAction>>   saisun_init_ac_;
    std::shared_ptr<actionlib::SimpleActionClient<saisun_msgs::TriggerAction>>   saisun_trig_ac_;
    std::shared_ptr<actionlib::SimpleActionClient<saisun_msgs::GetResultAction>> saisun_result_ac_;

    bool robot_pose_init_;

    std::string host_;
    unsigned int port_;
    std::string algorithm_version_;
    bool use_net_sequence_;

    saisun_msgs::RobotPose robot_pos_; 

    std::shared_ptr<std::thread> control_loop_thread_;

    void ros_init(void);
    void init(void);

    void action_start(receiveMessageTypes cmd, uint8_t *msg);
    void initial_ac_cb(const actionlib::SimpleClientGoalState& state,
                       const saisun_msgs::InitialResultConstPtr& result);
    void trigger_ac_cb(const actionlib::SimpleClientGoalState& state,
                       const saisun_msgs::TriggerResultConstPtr& result);
    void result_ac_cb(const actionlib::SimpleClientGoalState& state,
                      const saisun_msgs::GetResultResultConstPtr& result);

    void parse_robot_pos(void);
    void publish_msgs(void);
    void control_loop(void);

    void wait_vision_node(void);
};


#endif