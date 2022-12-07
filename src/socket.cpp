/*
   Author: Sherman Chen
   Create Time: 2022-12-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h> 
#include <netinet/in.h>

using namespace vsn;
using namespace socket;

// ref :
//    https://www.geeksforgeeks.org/socket-programming-cc/

#define BUF_LEN 1024
namespace {
    struct LCfg{

    }; LCfg lc_;
    
    
}

//------
void Node::read_loop()
{
    char buffer[BUF_LEN] = { 0 };
    int valread =0;
    while(valread >=0)
    {
        valread = ::read(cntx_.cur_socket, buffer, 1024);
        //printf("%s\n", buffer);
        //send(new_socket, hello, strlen(hello), 0);
        //printf("Hello message sent\n");
        log_d(" server read bytes "+str(valread));
        if(f_rcv_!=nullptr)
            f_rcv_(buffer, valread);
        sys::sleepMS(10);
    }
}
//------
void Node::send(const char* buf, int len)
{
    if(!cntx_.bConnected) return;
    ::send(cntx_.cur_socket, buf, len, 0);

}

//------
bool Server::run_thd()
{

    int server_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    auto& sock = cntx_.cur_socket;
    // Creating socket file descriptor
    server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_e("Server socket failed");
        return false;
    }
 
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        log_e("Socket server setsockopt() failed");
        return false;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(cntx_.port);
 
    // Forcefully attaching socket to the port 8080
    if (::bind(server_fd, (struct sockaddr*)&address,
             sizeof(address))
        < 0) {
        log_e("Socket server bind failed");
        return false;
    }

    //---- connection loop
    while(1)
    {
            
        cntx_.bConnected = false;
        if (::listen(server_fd, 1) < 0) {
            log_e("Server listen failed");
            break;
        }

        //---- wait connection
        log_i("Server wait connection on port "+
                    str(cntx_.port)+"...");
        if ((cntx_.cur_socket
            = ::accept(server_fd, (struct sockaddr*)&address,
                    (socklen_t*)&addrlen))
            < 0) {
            log_e("Server error when accept connection.");
            break;
        }

        //---- connected
        cntx_.bConnected = true;
        log_i("Connected with client, socket="+
            str(sock));

        //----- read loop
        read_loop();

        //---- when loop done,
        // closing the connected socket
        log_i("Disconnected with client");
        log_i("  Socket closed: "+str(sock));
        ::close(sock);
    }
    cntx_.isRunning = false;
    // closing the listening socket
    ::shutdown(server_fd, SHUT_RDWR);
    log_i("Server shutdown");
    return true;
}

//------
bool Server::start(int port)
{
    cntx_.port = port;
    cntx_.isRunning = true;
    //---- main connection loop thread
    thd_ = std::thread([&](){
        run_thd();
    });
    thd_.join();
}

//----
void Server::close()
{
    
}

//---------- Client
void Client::run_thd()
{
    cntx_.bConnected = false;

	int client_fd;
	struct sockaddr_in serv_addr;

    auto& sock = cntx_.cur_socket;
    sock = ::socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		log_e("Client socket creation failed");
		return;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(cntx_.port);

    const auto& sHost = cntx_.sHost;
	// Convert IPv4 and IPv6 addresses from text to binary
	// form
	if (inet_pton(AF_INET, sHost.c_str(), &serv_addr.sin_addr)
		<= 0) {
		log_e("Invalid host address:'"+sHost+"'");
		return;
	}
    //------
	if ((client_fd
		= ::connect(sock, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		log_e("Failed to connect to:'"+sHost+"' : "+str(cntx_.port));
		return;
	}
    //---- Connected
    cntx_.bConnected = true;
	
    //---- read loop
    read_loop();

    //--- exit
    log_i("Socket client disconnected");
    cntx_.bConnected = false;

	// closing the connected socket
	close(client_fd);
}




//----
bool Client::connect(const string& sHost, int port)
{
    cntx_.port = port;
    cntx_.sHost = sHost;
    thd_ = std::thread([&](){
        run_thd();
    });
    thd_.join();

    return true;
}

