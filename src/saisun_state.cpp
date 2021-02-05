#include "saisun_driver/saisun_state.h"

SaisunState::SaisunState(std::string host, unsigned int port)
{
    robot_version_.major_version = 0;
    robot_version_.minor_version = 0;
    use_net_sequence_ = false;

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

    uint8_t temp[buf_len];
    unsigned int len;
    memcpy(&len,&buf[offset],buf_len);
    if (use_net_sequence_)
    {
        len = ntohl(len);
    }
    memcpy(&temp[0],&len,sizeof(buf_len));
    

    
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

bool SaisunState::get_robot_state(void)
{
    // ROS_INFO("Trying send cmd to get robot state");
    return true;
}

receiveMessageTypes SaisunState::get_robot_cmd_(void)
{
    return receive_type_;
}