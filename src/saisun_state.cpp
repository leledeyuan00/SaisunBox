#include "saisun_driver/saisun_state.h"

SaisunState::SaisunState(std::string host, unsigned int port)
{
    robot_version_.major_version = 0;
    robot_version_.minor_version = 0;
    use_net_sequence_ = false;

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
    unsigned int len;
    memcpy(&len,&buf[offset],buf_len);
    if (use_net_sequence_)
    {
        len = ntohl(len);
    }
    memcpy(&temp[0],&len,sizeof(buf_len));
    
    if (temp[1] == 0x01) // robot req
    {
        switch ((receiveMessageTypes)temp[0])
        {
        case receiveMessageTypes::GET_SYNC:
        {
            heart_beat();
            break;
        }
        case receiveMessageTypes::GET_POSE:
        {
            offset = 16;
            while ((offset + sizeof(len)) <= buf_len)
            {
                
            }
            
            break;
        }
        default:
        {
            memcpy(receive_body_,&buf[16],(buf_len - 16));
            is_new_message_ = true;
            break;
        }
        }
    }
    
    
    // head
    // unsigned int len;
    // unsigned char message_type;
    // memcpy(&len,&buf[offset],sizeof(len));
    // len = ntohl(len);
    // offset += sizeof(len);

    // // ver
    // memcpy(&len,&buf[offset],sizeof(len));
    // len = ntohl(len);
    // offset += sizeof(len);

    // // Hex
    // memcpy(&len,&buf[offset],sizeof(len));
    // len = ntohl(len);
    // offset += sizeof(len);
    for (size_t i = 0; i < sizeof(len); i++)
    {
        ROS_INFO("heard %x",temp[i]);
    }
    if (use_net_sequence_)
    {
        memcpy(&len,&temp[0],sizeof(len));
        len = htonl(len);
        memcpy(&temp[0],&len,sizeof(len));
    }
    
    saisunCom_->write_sock(temp,buf_len);
    ROS_INFO("algorithm version is %x . %x",algorithm_version_.major_version,algorithm_version_.minor_version);

    ROS_INFO("I recieved data");
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