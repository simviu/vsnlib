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

//------
void Server::run_thd()
{

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = { 0 };
    char* hello = "Hello from server";
 
    // Creating socket file descriptor
    server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
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
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket
         = accept(server_fd, (struct sockaddr*)&address,
                  (socklen_t*)&addrlen))
        < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    //----- read
    valread = read(new_socket, buffer, 1024);
    printf("%s\n", buffer);
    send(new_socket, hello, strlen(hello), 0);
    printf("Hello message sent\n");

    //----
    // closing the connected socket
    ::close(new_socket);
    // closing the listening socket
    ::shutdown(server_fd, SHUT_RDWR);
}
//------
void Server::start(int port)
{
    cntx_.port = port;
    thd_ = std::thread([&](){
        run_thd();
    });
    thd_.join();
}

//----
void Server::close()
{
    
}

//------

