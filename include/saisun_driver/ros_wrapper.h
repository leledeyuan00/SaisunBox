#ifndef ROS_WRAPPER_H_
#define ROS_WRAPPER_H_

#include "saisun_state.h"

class SaisunWrapper
{
public:
    SaisunWrapper(ros::NodeHandle &nh);

    void start(void);
    void halt(void);


private:
    std::shared_ptr<SaisunState> saisunState_;
    ros::NodeHandle nh_;

    std::string host_;
    unsigned int port_;
    std::string algorithm_version_;
    bool use_net_sequence_;

    std::shared_ptr<std::thread> control_loop_thread_;

    void ros_init(void);
    void init(void);

    void control_loop(void);
};


#endif