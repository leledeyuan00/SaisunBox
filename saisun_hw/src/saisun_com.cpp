#include "saisun_hw/saisun_com.h"

SaisunCom::SaisunCom(std::string host,unsigned int port)
	:host_(host),port_(port)
{
	bzero((char *) &sec_serv_addr_, sizeof(sec_serv_addr_));
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) {
		printf("ERROR opening socket sec_sockfd");
	}
	server_ = gethostbyname(host_.c_str());
	if (server_ == NULL) {
		printf("ERROR, unknown host");
	}
	sec_serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&sec_serv_addr_.sin_addr.s_addr, server_->h_length);
	sec_serv_addr_.sin_port = htons(port_);
	flag_ = 1;

	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
			sizeof(int));
	setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_,
			sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
			sizeof(int));
	fcntl(sockfd_, F_SETFL, O_NONBLOCK);
	connected_ = false;
	keepalive_ = false;
}

bool SaisunCom::start() {
	keepalive_ = true;

	fd_set writefds;
	struct timeval timeout;
	std::cout << "Saisun Robot: Connecting ..." << std::endl;
	connect(sockfd_, (struct sockaddr *) &sec_serv_addr_,
			sizeof(sec_serv_addr_));
	FD_ZERO(&writefds);
	FD_SET(sockfd_, &writefds);

	timeout.tv_sec = 10;
	timeout.tv_usec = 0;

	bool success = select(sockfd_ + 1, NULL, &writefds, NULL, &timeout) > 0 ? true : false;
	
	
	unsigned int flag_len;
	getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
	if (flag_ < 0) {
		std::cout <<"Error connecting to Saisun Robot" << std::endl;
		return false;
	}
	if (!success)
	{
		std::cout << "Saisun Robot: Connecting timeout, try to reconnect." << std::endl;
	}else
	{
		std::cout << "Saisun Robot: Got connection" << std::endl;
	}
		
	FD_ZERO(&readfds_);
	FD_SET(sockfd_, &readfds_);
	connected_ = true;
	return success;

}

void SaisunCom::halt() {
	keepalive_ = false;
	shutdown(sockfd_,SHUT_RDWR);
}

void SaisunCom::write_sock(uint8_t *buf, unsigned int buf_len)
{
	if (connected_)
	{
		write(sockfd_, buf, buf_len);
	}
}

bool SaisunCom::isAlive(void)
{
	return keepalive_;
}

bool SaisunCom::isConnect(uint8_t * buf,int * bytes_read)
{
	struct timeval timeout;
	
	if (connected_ && keepalive_)
	{
		timeout.tv_sec = 1; //do this each loop as selects modifies timeout
		timeout.tv_usec = 500000; // timeout of 0.5 sec
		select(sockfd_ + 1, &readfds_, NULL, NULL, &timeout);
		*bytes_read = read(sockfd_, buf, 2048); // usually only up to 1295 bytes
		if (*bytes_read > 0) {
			setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK,
					(char *) &flag_, sizeof(int));
			return true;
		} else {
			connected_ = false;
			close(sockfd_);
			return false;
		}
	}else{
		return false;
	}
}

void SaisunCom::reconnect(void)
{

	//reconnect
	std::cout << "Saisun Robot: No connection. Is controller crashed? Will try to reconnect in 5 seconds..." << std::endl;
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) {
		std::cout << ("ERROR opening Saisun Robot socket") << std::endl;
	}
	flag_ = 1;
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
			sizeof(int));
	setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_,
			sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
			sizeof(int));
	fcntl(sockfd_, F_SETFL, O_NONBLOCK);
	if (keepalive_ && !connected_) {
		std::this_thread::sleep_for(std::chrono::seconds(5)); // 5s dela
		fd_set writefds;

		connect(sockfd_, (struct sockaddr *) &sec_serv_addr_,
				sizeof(sec_serv_addr_));
		FD_ZERO(&writefds);
		FD_SET(sockfd_, &writefds);
		select(sockfd_ + 1, NULL, &writefds, NULL, NULL);
		unsigned int flag_len;
		getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_,
				&flag_len);
		if (flag_ < 0) {
			std::cout << ("Error re-connecting to port 9999. Is controller started? Will try to reconnect in 5 seconds...") << std::endl;
		} else {
			connected_ = true;
			FD_ZERO(&readfds_);
			FD_SET(sockfd_, &readfds_);
			std::cout << ("Saisun Robot: Reconnected") << std::endl;
		}
	}
}

// void SaisunCom::run(){
// 	std::cout << "start TCP connection" << std::endl;
// 	uint8_t buf[2048];
// 	int bytes_read;
// 	bzero(buf, 2048);
// 	struct timeval timeout;
// 	fd_set readfds;
// 	FD_ZERO(&readfds);
// 	FD_SET(sockfd_, &readfds);
// 	connected_ = true;
// 	while (keepalive_) {
// 		while (connected_ && keepalive_) {
// 			timeout.tv_sec = 1; //do this each loop as selects modifies timeout
// 			timeout.tv_usec = 500000; // timeout of 0.5 sec
// 			select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);
// 			bytes_read = read(sockfd_, buf, 2048); // usually only up to 1295 bytes
// 			if (bytes_read > 0) {
// 				setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK,
// 						(char *) &flag_, sizeof(int));
// 			} else {
// 				connected_ = false;
// 				close(sockfd_);
// 			}
// 		}
// 		if (keepalive_) {
// 			//reconnect
// 			std::cout << "Saisun Robot: No connection. Is controller crashed? Will try to reconnect in 10 seconds..." << std::endl;
// 			sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
// 			if (sockfd_ < 0) {
// 				std::cout << ("ERROR opening Saisun Robot socket") << std::endl;
// 			}
// 			flag_ = 1;
// 			setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
// 					sizeof(int));
// 			setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_,
// 					sizeof(int));
// 			setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
// 					sizeof(int));
// 			fcntl(sockfd_, F_SETFL, O_NONBLOCK);
// 			while (keepalive_ && !connected_) {
// 				std::this_thread::sleep_for(std::chrono::seconds(10)); // 10s delay
// 				fd_set writefds;

// 				connect(sockfd_, (struct sockaddr *) &sec_serv_addr_,
// 						sizeof(sec_serv_addr_));
// 				FD_ZERO(&writefds);
// 				FD_SET(sockfd_, &writefds);
// 				select(sockfd_ + 1, NULL, &writefds, NULL, NULL);
// 				unsigned int flag_len;
// 				getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_,
// 						&flag_len);
// 				if (flag_ < 0) {
// 					std::cout << ("Error re-connecting to port 9999. Is controller started? Will try to reconnect in 10 seconds...") << std::endl;
// 				} else {
// 					connected_ = true;
// 					FD_ZERO(&readfds);
// 					FD_SET(sockfd_, &readfds);
// 					std::cout << ("Saisun Robot: Reconnected") << std::endl;
// 				}
// 			}
// 		}
// 	}
// 	//wait for some traffic.
// 	std::this_thread::sleep_for(std::chrono::milliseconds(500));
// 	close(sockfd_);
// }
