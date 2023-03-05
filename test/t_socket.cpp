/*
   Author: Sherman Chen
   Create Time: 2022-12-06
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"

using namespace vsn;
using namespace ut;
using namespace test;

namespace{
    struct LCfg{
        int port = 8192;
        string host = "127.0.0.1";
    }; LCfg lc_;
}

//----
bool TestSocketSrvr::run()
{
    socket::Server srv;
    if(!srv.start(lc_.port))
        return false;

    int i=0;
    
    
    //----
    while(1)
    {
        string s;
        if(!srv.recvLn(s))
            break;
        log_i("Recv: '"+s+"'");
        //---- ack
        if(!srv.send("ack:"+to_string(i)))
            break;
        sys::sleepMS(500);
    }
    return true;
}

//------------
bool TestSocketClnt::run()
{
    socket::Client clnt;
    
    
    //------
    int i=0;
    bool ok = clnt.connect(lc_.host, lc_.port);

    while(ok)
    {

        string s = "hello "+to_string(i++)+"\n";
        log_i("send :'"+s+"'");
        clnt.send(s);
        log_i("  sent");
        //--
        log_i("recvLn...");
        if(!clnt.recvLn(s)) 
            break;
        
        log_i("  client recv:'"+s+"'");

        sys::sleepMS(1000);
    }
    return ok;
}

