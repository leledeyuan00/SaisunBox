#ifndef ROS_WRAPPER_H_
#define ROS_WRAPPER_H_

#include "saisun_state.h"

#include "geometry_msgs/Twist.h"

using namespace receive_message_types;
using namespace send_message_types;

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
    ros::NodeHandle nh_;

    ros::Publisher robot_pose_pub_;
    bool robot_pose_init_;

    std::string host_;
    unsigned int port_;
    std::string algorithm_version_;
    bool use_net_sequence_;

    geometry_msgs::Twist robot_pos_; 

    std::shared_ptr<std::thread> control_loop_thread_;

    void ros_init(void);
    void init(void);

    void action_start(void);

    void parse_robot_pos(void);
    void publish_msgs(void);
    void control_loop(void);
};


#endif