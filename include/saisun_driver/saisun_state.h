#ifndef SAISUN_STATE_H_
#define SAISUN_STATE_H_

#include "saisun_com.h"

#include <inttypes.h>
#include <vector>
#include <stdlib.h>
#include <mutex>
#include <condition_variable>
#include <netinet/in.h>
#include "ros/ros.h"

namespace receive_message_types{
enum receive_message_types{
    GET_SYNC = 0x20,
    GET_INIT = 0x21,
    GET_TRIG = 0x24,
    GET_DATA = 0x23,
    GET_POSE = 0x40,
};
}
typedef receive_message_types::receive_message_types receiveMessageTypes;

namespace send_message_types{
enum send_message_types{
    SEND_SYNC = 0x50,
    SEND_INIT = 0x51,
    SEND_TRIG = 0x54,
    SEND_DATA = 0x53,
    SEND_POSE = 0x30,
};
}
typedef send_message_types::send_message_types sendMessageTypes;

namespace vision_state_types{
enum vision_state_types{
    VISION_OK    = 0x00,
    VISION_ERROR = 0x01,
    CAMERA_ERROR = 0x02,
    OTHER_ERROR  = 0xff,
};
}
typedef vision_state_types::vision_state_types visionStateTypes;

struct version_message{
    uint16_t major_version;
    uint16_t minor_version;
};

class SaisunState
{
private:

    version_message robot_version_;
    version_message algorithm_version_;
    bool use_net_sequence_;

    receiveMessageTypes receive_type_;
    sendMessageTypes send_type_;
    visionStateTypes vision_state_;

    std::thread comThread_;

    std::vector<float> robot_pose_;

    bool is_new_message_;
    uint8_t receive_body_[8];

    void robot_pose_cmd(void);
    void heart_beat(void);
    void run();

public:
    std::shared_ptr<SaisunCom> saisunCom_;

    SaisunState(std::string host, unsigned port);
    void unpack(uint8_t * buf,unsigned int buf_len);

    bool set_algorithm_version(std::string ver);
    void set_use_net_sequence(bool isuse);
    void get_robot_state(std::vector<float> &robot_pose);

    bool get_robot_cmd_(receiveMessageTypes &cmd, uint8_t *msg);

    void start();
    void halt();
    // virtual void req_messure(uint8_t method, uint8_t * res);
};


#endif