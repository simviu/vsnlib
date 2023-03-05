/*
   Author: Sherman Chen
   Create Time: 2022-12-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "ut/cutil.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h> 
#include <netinet/in.h>

using namespace ut;
using namespace socket;

// ref :
//    https://www.geeksforgeeks.org/socket-programming-cc/

#define BUF_LEN 1024
namespace {
    struct LCfg{

    }; LCfg lc_;
    
    //---- Linux C read line
    // Ref: https://www.man7.org/tlpi/code/online/dist/sockets/read_line.c.html
    ssize_t
    c_readLine(int fd, char *buffer, size_t n)
    {
        ssize_t numRead;                    /* # of bytes fetched by last read() */
        size_t totRead;                     /* Total bytes read so far */
        char *buf;
        char ch;

        if (n <= 0 || buffer == NULL) {
            errno = EINVAL;
            return -1;
        }

        buf = buffer;                       /* No pointer arithmetic on "void *" */

        totRead = 0;
        for (;;) {
            numRead = ::recv(fd, &ch, 1, MSG_WAITALL);

            if (numRead == -1) {
                if (errno == EINTR)         /* Interrupted --> restart read() */
                    continue;
                else
                    return -1;              /* Some other error */

            } else if (numRead == 0) {      /* EOF */
                if (totRead == 0)           /* No bytes read; return 0 */
                    return 0;
                else                        /* Some bytes read; add '\0' */
                    break;

            } else {                        /* 'numRead' must be 1 if we get here */
                if (totRead < n - 1) {      /* Discard > (n - 1) bytes */
                    totRead++;
                    *buf++ = ch;
                }

                if (ch == '\n' || ch=='\r' )
                    break;
            }
        }

        *buf = '\0';
        return totRead;
    }
        
}
//-----
void Node::onDisconnect()
{
    cntx_.bConnected = false;
    //---- when loop done,
    // closing the connected socket
    log_i("Disconnected with client");
    int sock = cntx_.cur_socket;
    log_i("  Socket closed: "+to_string(sock));
    ::close(sock);

}
//------
bool Node::recvLn(string& sln)
{
    std::unique_lock<std::mutex> lk(rd_mtx_);
    if(!cntx_.bConnected) 
        return false;

    sln = "";
    char buffer[BUF_LEN] = { 0 };
    ssize_t n = c_readLine(cntx_.cur_socket, buffer, 1024);
    
    if(n<=0){
        onDisconnect();
        return false;
    }

    sln = string(buffer, n);
    return true;
}
//------
bool Node::send(const string& s)
{ 
    uint8_t* p = (uint8_t*)(s.c_str());
    const Buf buf(p, s.length());
    return send(buf); 
}
//----
bool Node::recv(Buf& buf)
{
    char* p = (char*)buf.p;
    int fd = cntx_.cur_socket;
    size_t n = ::recv(fd, p, buf.n, MSG_WAITALL);
    return n == buf.n;

}

//------
bool Node::send(const Buf& buf)
{
    std::unique_lock<std::mutex> lk(wr_mtx_);
    if(!cntx_.bConnected) 
        return false;
    
    size_t n = ::send(cntx_.cur_socket, buf.p, buf.n, 0);
    if(n<0){
        onDisconnect();
        return false;
    }
    if(n!=buf.n)
    {
        log_e("Send length differ");
        onDisconnect();
        return false;
    }
    return true;

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
        //read_loop(cntx);
        while(cntx.bConnected)
            sys::sleepMS(100);

        // when disconnected , 
        // loop over next connection.
        
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
		log_e("Failed to connect to:'"+sHost+"' : "+
            to_string(cntx_.port));
		return false;
	}

    //---- Connected
    log_i("Socket client connected");
    cntx_.bConnected = true;

    return true;
}





