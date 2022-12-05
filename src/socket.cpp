/*
   Author: Sherman Chen
   Create Time: 2022-12-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"

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
void Server::listen_thd()
{

    int server_fd, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[BUF_LEN] = { 0 };

    // Creating socket file descriptor
    server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_e("Server socket failed");
        return;
    }
 
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(cntx_.port);
 
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
             sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    //----- main loop
    while(1)
    {
        auto& new_socket = cntx_.cur_socket;
        if (listen(server_fd, 1) < 0) {
            log_e("Server listen failed");
            return;
        }
        
        //---- wait connection
        log_i("Server wait connection on port "+
                    str(cntx_.port)+"...");
        if ((new_socket
            = accept(server_fd, (struct sockaddr*)&address,
                    (socklen_t*)&addrlen))
            < 0) {
            log_e("Server error when accept connection.");
        }
        log_i("Connected with client, socket="+str(new_socket));
        //----- read loop
        while(valread >=0)
        {
            valread = read(new_socket, buffer, 1024);
            //printf("%s\n", buffer);
            //send(new_socket, hello, strlen(hello), 0);
            //printf("Hello message sent\n");
            log_d(" server read bytes "+str(valread));
            sys::sleepMS(10);
        }
        //----
        // closing the connected socket
        log_i("Disconnected with client");
        ::close(new_socket);
    }
    // closing the listening socket
    ::shutdown(server_fd, SHUT_RDWR);
    log_i("Socket closed");

}
//------
void Server::start(int port)
{
    cntx_.port = port;
    thd_ = std::thread([&](){
        listen_thd();
    });
    thd_.join();
}

//----
void Server::close()
{
    
}

//------

