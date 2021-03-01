#ifndef SAISUN_COM_H_
#define SAISUN_COM_H_

// #include "saisun_state.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <fcntl.h>
#include <boost/shared_ptr.hpp>

#include "saisun_hw/visibiblity_control.h"

class SaisunCom
{
private:
	// TCP protocol
    int sockfd_;
	struct sockaddr_in pri_serv_addr_, sec_serv_addr_;
	struct hostent *server_;
	bool keepalive_;
	int flag_;
	std::string host_;
	unsigned int port_;

	fd_set readfds_;
	void run();


public:
	bool connected_;

    SaisunCom(std::string host,unsigned int port);
	bool start();
	void halt();
	void write_sock(uint8_t *buf, unsigned int buf_len);

	bool isConnect(uint8_t * buf,int *bytes_read);
	bool isAlive(void);
	bool reconnect(void);
};


#endif