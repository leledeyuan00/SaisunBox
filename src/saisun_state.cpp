#include "saisun_driver/saisun_state.h"

using namespace receive_message_types;
using namespace send_message_types;
using namespace vision_state_types;

SaisunState::SaisunState(std::string host, unsigned int port)
{
    robot_version_.major_version = 0;
    robot_version_.minor_version = 0;
    use_net_sequence_ = false;

    vision_state_ = VISION_OK;

    robot_pose_.resize(6);

    saisunCom_.reset(new SaisunCom(host,port));
}

void SaisunState::start(void)
{
    saisunCom_->start();
    comThread_ = std::thread(&SaisunState::run, this);
}

void SaisunState::run(void)
{
    uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);
    while (saisunCom_->isAlive())
    {
        if (saisunCom_->isConnect(buf,&bytes_read))
        {
            unpack(buf,bytes_read);
        }
        else if (saisunCom_->isAlive())
        {
            saisunCom_->reconnect();
        }
    }
    // wait for some traffic
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void SaisunState::halt(void)
{
    saisunCom_->halt();
    comThread_.join();
}

void SaisunState::unpack(uint8_t * buf, unsigned int buf_len)
{
    receiveMessageTypes receive_type;
    sendMessageTypes send_type;
    unsigned int offset = 0;
    bzero(receive_body_,8);

    is_new_message_ = true;

    uint8_t temp[buf_len];
    memcpy(&temp,buf,buf_len);

    if (use_net_sequence_)
    {
        uint32_t len;
        uint32_t offset= 0;
        while (offset + sizeof(len) <= buf_len)
        {
            memcpy(&len,&temp[offset],sizeof(len));
            len = htonl(len);
            memcpy(&temp[offset],&len,sizeof(len));
            offset += sizeof(len);
        }
    }
    uint16_t check_bytes = 0x0001;
    uint16_t receive_check_bytes;
    memcpy(&receive_type_,&temp[10],sizeof(check_bytes));
    if (receive_type_ != check_bytes)
    {
        ROS_WARN("Receive an irregular message");
        return;
    }
    
    if (temp[8] == 0x01) // robot req
    {
        switch ((receiveMessageTypes)temp[9])
        {
        case receiveMessageTypes::GET_SYNC:
        {
            heart_beat();
            break;
        }
        case receiveMessageTypes::GET_POSE:
        {
            
            break;
        }
        default:
        {
            memcpy(receive_body_,&temp[16],(buf_len - 16));
            is_new_message_ = true;
            break;
        }
        }
    }
}

void SaisunState::heart_beat(void)
{
    uint8_t vision[4] = {
        (uint8_t)(robot_version_.minor_version >> 8 &0xff),
        (uint8_t)(robot_version_.minor_version >> 0 &0xff),
        (uint8_t)(robot_version_.major_version >> 8 &0xff),
        (uint8_t)(robot_version_.major_version >> 0 &0xff),
    };
    uint8_t temp[17] = {
        0x00,0x00,0x00,0x00,
        vision[0],vision[1],vision[2],vision[3],
        0x80,SEND_SYNC,0x00,0x00,
        0x01,0x00,0x00,0x00,
        (uint8_t)vision_state_
    };
    if (use_net_sequence_)
    {
        uint32_t len;
        uint32_t offset= 0;
        while (offset + sizeof(len) <= 17)
        {
            memcpy(&len,&temp[offset],sizeof(len));
            len = htonl(len);
            memcpy(&temp[offset],&len,sizeof(len));
            offset += sizeof(len);
        }
    }
    saisunCom_->write_sock(temp,17);
}

bool SaisunState::set_algorithm_version(std::string ver)
{
    std::size_t pos_dot = ver.find(".");
    if (pos_dot != 0)
    {
        algorithm_version_.major_version = (uint16_t) atoi(ver.substr(0,pos_dot).c_str()); 
        algorithm_version_.minor_version = (uint16_t) atoi(ver.substr(pos_dot+1).c_str()); 
        return true;
    }else{
        return false;
    }
}

void SaisunState::set_use_net_sequence(bool isuse)
{
    use_net_sequence_ = isuse;
}

void SaisunState::get_robot_state(std::vector<float> &robot_pose)
{
    for (size_t i = 0; i < sizeof(robot_pose_); i++)
    {
        robot_pose[i] = robot_pose_[i];
    }
    return;
}

bool SaisunState::get_robot_cmd_(receiveMessageTypes &cmd, uint8_t *msg)
{
    bool is_new_message = is_new_message_;
    is_new_message_ = false;
    if(is_new_message)
    {
        cmd = receive_type_;
        memcpy(msg,receive_body_,8);
    }
    return is_new_message;
}