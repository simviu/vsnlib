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
    srv.setRcv([&](const char* buf, int len){
        string s(buf, len);
        log_i("  server recv:'"+s+"'");
        //---- echo back
        string s1 = "ack "+to_string(i++);
        srv.send(s1);
    });

    //----
    while(1)
        sys::sleepMS(500);
    return true;
}

//------------
bool TestSocketClnt::run()
{
    socket::Client clnt;
    clnt.setRcv([&](const char* buf, int len){
        string s(buf, len);
        log_i("  client recv:'"+s+"'");
    });
    //------
    int i=0;
    bool ok = clnt.connect(lc_.host, lc_.port);

    while(ok)
    {

        string s = "hello "+to_string(i++);
        log_i("send :'"+s+"'");
        clnt.send(s);
        sys::sleepMS(1000);
    }
    return ok;
}

