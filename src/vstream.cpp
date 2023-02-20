/*
   Author: Sherman Chen
   Create Time: 2023-02-19
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLibCv.h"


using namespace vsn;
using namespace vstream;

namespace{
    struct LCfg{
        float t_loop_delay = 0.001;
    }; LCfg lc_;
}

//----

bool Server::init(int port)
{
    log_i("Start vstream server at port "+to_string(port));
    bool ok = svr_.start(port);
    if(!ok) log_e(" vstream server failed");

    if(!ok) return false;

    //--- start loop thread
    thd_ = std::thread([&](){
        run_loop();
    });
    thd_.detach();

    return true;
}
//----
bool Server::open(const string& sf)
{
    p_video_ = Video::create(cam_id);
    bool ok = (p_video_!=nullptr);
    string s = (ok?"OK ":"Failed") + " to vstream server open video:'"+sf+"'"; 
    if(ok) log_i(s);
    else log_e(s);
    return ok;

}
//----
bool Server::open(int cam_id)
{
    p_video_ = Video::create(cam_id);
    if(p_video_!=nullptr)
    {
        log_i("vstream server openned camera:"+to_string(cam_id));
        return true;
    }
    log_e("vstream server failed to open camera:"+to_strimg(cam_id));
    return false;

}

//----
void Server::run_loop()
{
    while(1)
    {
        run_once();
        sys::sleep(lc_.t_loop_dely);
    }
}

//----
void Server::run_once()
{
    if(p_video_==nullptr)
        return;
    auto& vd = *p_video_;
    auto p = vd.read();
    push(p);

}
//----
void Server::push(Sp<Img> p)
{
    cv::Mat im = img2cv(*p);
    
}
