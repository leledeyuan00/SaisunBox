#ifndef SAISUN_STATE_H_
#define SAISUN_STATE_H_

#include "saisun_com.h"

#include <inttypes.h>
#include <vector>
#include <stdlib.h>
#include <mutex>
#include <condition_variable>
#include <netinet/in.h>

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
    SEND_ERROR= 0XFF,
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

struct versionMessage{
    uint16_t major_version;
    uint16_t minor_version;
    uint8_t vision_8[4];
};

struct reqPoseType{
    uint8_t state;
    uint8_t frame_type;
    uint8_t ex_joint_num;
};

class SaisunState
{
private:

    versionMessage robot_version_;
    versionMessage algorithm_version_;
    reqPoseType req_pose_type_;
    bool use_net_sequence_;

    receiveMessageTypes receive_type_;
    sendMessageTypes send_type_;
    visionStateTypes vision_state_;

    std::thread comThread_;
    std::thread posThread_;

    std::vector<float> robot_pose_;

    bool is_new_message_;
    bool is_new_pos_;
    bool is_init_pos_;
    uint8_t receive_body_[8];

    void robot_pose_req(void);
    void robot_pose_res(uint8_t *buf_body, uint32_t buf_len);
    void unpack(uint8_t * buf,uint32_t buf_len);
    void run();

public:
    std::shared_ptr<SaisunCom> saisunCom_;

    void pack(sendMessageTypes cmd,uint8_t *body, uint32_t body_len);
    SaisunState(std::string host, uint32_t port);

    bool set_algorithm_version(std::string ver);
    void set_req_pose_type(reqPoseType req);
    void set_use_net_sequence(bool isuse);
    void set_vision_state(visionStateTypes vs);

    bool get_robot_pose(std::vector<float> &robot_pose);
    bool get_robot_cmd(receiveMessageTypes &cmd, uint8_t *msg);

    void start();
    void halt();
};


#endif