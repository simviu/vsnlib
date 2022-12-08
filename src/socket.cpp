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
void read_loop(Node::Cntx& cntx)
{
    char buffer[BUF_LEN] = { 0 };

    while(1)
    {
        int n = ::read(cntx.cur_socket, buffer, 1024);
        //printf("%s\n", buffer);
        //send(new_socket, hello, strlen(hello), 0);
        //printf("Hello message sent\n");
        //log_d("  recv bytes "+to_string(valread));
        if(n>0 && cntx.f_rcv_!=nullptr)
            cntx.f_rcv_(buffer, n);
        sys::sleepMS(10);
    }
}
//------
bool Node::send(const char* buf, int len)
{
    if(!cntx_.bConnected) 
        return false;
    
    size_t n = ::send(cntx_.cur_socket, buf, len, 0);

    return (n==len);

}

//------
bool server_thd(Node::Cntx& cntx)
{

    int server_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    auto& sock = cntx.cur_socket;
    //int sock=0;
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
    address.sin_port = htons(cntx.port);
 
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
            
        cntx.bConnected = false;
        if (::listen(server_fd, 1) < 0) {
            log_e("Server listen failed");
            break;
        }

        //---- wait connection
        log_i("Server wait connection on port "+
                    to_string(cntx.port)+"...");
        if ((cntx.cur_socket
            = ::accept(server_fd, (struct sockaddr*)&address,
                    (socklen_t*)&addrlen))
            < 0) {
            log_e("Server error when accept connection.");
            break;
        }

        //---- connected
        cntx.bConnected = true;
        log_i("Connected with client, socket="+
            to_string(sock));

        //----- read loop
        read_loop(cntx);

        //---- when loop done,
        // closing the connected socket
        log_i("Disconnected with client");
        log_i("  Socket closed: "+to_string(sock));
        ::close(sock);
    }
    cntx.isRunning = false;
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
        server_thd(cntx_);
    });
    thd_.detach();
    
    return true;
}

//----
void Server::close()
{
    
}

//----
bool Client::connect(const string& sHost, int port)
{
    log_i("Socket client connect to "+
        sHost + " : "+to_string(port) + " ...");
    cntx_.port = port;
    cntx_.sHost = sHost;
    
    cntx_.bConnected = false;

	int client_fd;
	struct sockaddr_in serv_addr;

    auto& sock = cntx_.cur_socket;
    sock = ::socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		log_e("Client socket creation failed");
		return false;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(cntx_.port);

	// Convert IPv4 and IPv6 addresses from text to binary
	// form
	if (inet_pton(AF_INET, sHost.c_str(), &serv_addr.sin_addr)
		<= 0) {
		log_e("Invalid host address:'"+sHost+"'");
		return false;
	}
    //------
	if ((client_fd
		= ::connect(sock, (struct sockaddr*)&serv_addr,
				sizeof(serv_addr)))
		< 0) {
		log_e("Failed to connect to:'"+sHost+"' : "+str(cntx_.port));
		return false;
	}
    log_i("Socket client connected");

    //---- read thread
    thd_ = std::thread([&](){
        read_loop(cntx_);


        //--- exit
        log_i("Socket client disconnected");
        cntx_.bConnected = false;

        // closing the connected socket
        close(client_fd);

    });
    thd_.detach();
   //---- Connected
    cntx_.bConnected = true;
    log_i("Client ready");

    return true;
}





