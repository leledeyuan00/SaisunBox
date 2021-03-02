#include "saisun_hw/saisun_state.h"

using namespace receive_message_types;
using namespace send_message_types;
using namespace vision_state_types;

SaisunState::SaisunState(std::string host, uint32_t port)
{
    robot_version_.major_version = 0;
    robot_version_.minor_version = 0;
    use_net_sequence_ = false;
    is_init_pos_ = false;
    // default request pose types
    req_pose_type_.state = 0x00;
    req_pose_type_.frame_type = 0x00;
    req_pose_type_.ex_joint_num = 0x00;

    vision_state_ = OTHER_ERROR;

    robot_pose_.resize(6);

    saisunCom_.reset(new SaisunCom(host,port));
}

void SaisunState::start(void)
{
    saisunCom_->start();
    comThread_ = std::thread(&SaisunState::run, this);
    posThread_ = std::thread(&SaisunState::robot_pose_req,this);
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

void SaisunState::robot_pose_req(void)
{
    uint32_t time_out = 0;
    is_init_pos_ = true;
    is_new_pos_ = true;
    while (saisunCom_->isAlive())
    {
        if (is_new_pos_)
        {
            uint8_t body[3] = {
                req_pose_type_.state, req_pose_type_.frame_type, req_pose_type_.ex_joint_num};
            pack(SEND_POSE,body,3);

            time_out = 0;
            is_new_pos_ = false;
        }
        if (time_out >= 100)
        {
            // resend pose req
            time_out = 0;
            is_new_pos_ = true;
        }
        time_out ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SaisunState::robot_pose_res(uint8_t *buf_body, uint32_t buf_len)
{
    is_new_pos_ = true;
    uint32_t offset = 0;
    uint32_t len;
    float temp_f;

    if (buf_body[0] != 0x00)
    {
        std::cout << "error robot pose" << std::endl;
        return;
    }
    if ((robot_pose_.size()*4+4) > buf_len)
    {
        std::cout << "error robot buffer len" << std::endl;
    }
    
    offset += sizeof(len);

    for (size_t i = 0; i < robot_pose_.size(); i++)
    {
        memcpy(&len,&buf_body[offset],sizeof(len));
        memcpy(&temp_f,&len,sizeof(len));
        robot_pose_[i] = temp_f;
        offset += sizeof(len);
    }
}

void SaisunState::halt(void)
{
    saisunCom_->halt();
    comThread_.join();
    posThread_.join();
}

void SaisunState::unpack(uint8_t * buf, uint32_t buf_len)
{
    const size_t max_buff_len = 16 + 28;
    bzero(receive_body_,8);

    is_new_message_ = true;
    
    uint32_t net_com_len = ((buf_len % 4) != 0) ? (((buf_len >> 2) +1) *4) : buf_len;

    uint8_t temp[max_buff_len];
    memset(temp,0,sizeof(max_buff_len));
    memcpy(&temp,buf,buf_len);

    if (use_net_sequence_)
    {
        uint32_t len;
        uint32_t offset= 0;
        while (offset + sizeof(len) <= net_com_len)
        {
            memcpy(&len,&temp[offset],sizeof(len));
            len = htonl(len);
            memcpy(&temp[offset],&len,sizeof(len));
            offset += sizeof(len);
        }
    }

    memcpy(&robot_version_.major_version,&temp[6],sizeof(uint16_t));
    memcpy(&robot_version_.minor_version,&temp[4],sizeof(uint16_t));

    uint16_t check_bytes = 0x0001;
    uint16_t receive_check_bytes;
    memcpy(&receive_check_bytes,&temp[10],sizeof(check_bytes));
    if (receive_check_bytes != check_bytes)
    {
        std::cout << "\033[33m" << "Receive an irregular message. Receive type is 0x"<< receive_check_bytes <<", try switch net_sequence in config.yaml" << "\033[0m" <<std::endl;
        return;
    }
    if (temp[8] == 0x01) // robot req
    {
        uint32_t receive_buf_len;
        memcpy(&receive_buf_len,&temp[12],sizeof(receive_buf_len));
        if (receive_buf_len != (buf_len - 16))
        {
            std::cout << receive_buf_len << "ERROR BUFLEN" << (buf_len - 16) << std::endl;
            return;
        }
        receive_type_ = (receiveMessageTypes)temp[9];
        switch (receive_type_)
        {
        case receiveMessageTypes::GET_SYNC:
        {
            uint8_t body = (uint8_t)vision_state_;
            pack(SEND_SYNC,&body,1);
            break;
        }
        case receiveMessageTypes::GET_POSE:
        {
            robot_pose_res(&temp[16],(net_com_len - 16));
            break;
        }
        default:
        {
            memcpy(receive_body_,&temp[16],(net_com_len - 16));
            is_new_message_ = true;
            break;
        }
        }
    }
}
void SaisunState::pack(sendMessageTypes cmd,uint8_t *body, uint32_t body_len)
{
    const size_t max_buff_len = 8*64 + 4;
    uint32_t head_len;
    uint32_t all_len = 4 * sizeof(head_len) + body_len;
    uint8_t vision[4] = {
    (uint8_t)(algorithm_version_.minor_version >> 8 &0xff),
    (uint8_t)(algorithm_version_.minor_version >> 0 &0xff),
    (uint8_t)(algorithm_version_.major_version >> 8 &0xff),
    (uint8_t)(algorithm_version_.major_version >> 0 &0xff),
    };
    uint8_t temp[max_buff_len] = {
        0x00,0x00,0x00,0x00, // fixed head
        vision[0],vision[1],vision[2],vision[3],
        0x80,(uint8_t)cmd,0x00,0x00,
    };
    memcpy(&temp[3*sizeof(head_len)],&body_len,sizeof(body_len));
    memcpy(&temp[4*sizeof(head_len)],body,body_len);
    
    if (use_net_sequence_)
    {
        uint32_t len;
        uint32_t offset= 0;
        while (offset + sizeof(len) <= all_len)
        {
            memcpy(&len,&temp[offset],sizeof(len));
            len = htonl(len);
            memcpy(&temp[offset],&len,sizeof(len));
            offset += sizeof(len);
        }
    }    
    
    saisunCom_->write_sock(temp, all_len);
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
        std::cout << "Please check version should be `xx.xx` format" << std::endl;
        return false;
    }
}

void SaisunState::set_use_net_sequence(bool isuse)
{
    use_net_sequence_ = isuse;
}

void SaisunState::set_req_pose_type(reqPoseType req)
{
    req_pose_type_.state = req.state;
    req_pose_type_.frame_type = req.frame_type;
    req_pose_type_.ex_joint_num = req.ex_joint_num;
}

bool SaisunState::get_robot_pose(std::vector<float> &robot_pose)
{
    for (size_t i = 0; i < robot_pose_.size(); i++)
    {
        robot_pose[i] = robot_pose_[i];
    }
    return is_init_pos_;
}

bool SaisunState::get_robot_cmd(receiveMessageTypes &cmd, uint8_t *msg)
{
    bool is_new_message = is_new_message_;
    if(is_new_message)
    {
        cmd = receive_type_;
        memcpy(msg,receive_body_,8);
        is_new_message_ = false;
    }
    return is_new_message;
}

void SaisunState::set_vision_state(visionStateTypes vs)
{
    vision_state_ = vs;
}