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
    p_video_ = Video::open(sf);
    bool ok = (p_video_!=nullptr);
    string s = (ok?"OK ":"Failed");
    s  += " to vstream server open video:'"+sf+"'"; 
    if(ok) log_i(s);
    else log_e(s);
    return ok;

}
//----
bool Server::open(int cam_id)
{
    p_video_ = Video::open(cam_id);
    if(p_video_!=nullptr)
    {
        log_i("vstream server openned camera:"+to_string(cam_id));
        return true;
    }
    log_e("vstream server failed to open camera:"+to_string(cam_id));
    return false;

}

//----
void Server::run_loop()
{
    while(1)
    {
        run_once();
        sys::sleep(lc_.t_loop_delay);
    }
}

//----
void Server::run_once()
{
    if(p_video_==nullptr)
        return;
    auto& vd = *p_video_;
    auto p = vd.read();
    send(p);

}
//----
void Server::send(Sp<Img> p)
{
    cv::Mat im = img2cv(*p);
    vector<uchar> buf;
    cv::imencode(".bmp", im, buf);

    //---- to binary
    auto pb = (uint8_t*)(&buf[0]);
    int n  = buf.size();
    Buf b(pb, n);
    svr_.send(b);
}
//----
bool Client::connect(const string& sHost, int port)
{
    log_i("vstream Client init...");
    if(!clnt_.connect(sHost, port))
        return false;
    
    return true;
}

//----
void Client::run_loop()
{
    while(1)
    {
        if(!clnt_.isRunning()) 
            break;

        run_once();
        sys::sleep(lc_.t_loop_delay);
    }
    log_i("vstream client disconnected");
}

//----
bool Client::run_once()
{
    Buf buf;
    if(!clnt_.read(buf))
        return false;
    vector<uchar> ds;
    for(int i=0;i<buf.n;i++)
        ds.push_back(buf.p[i]);
    cv::Mat im  = cv::imdecode(ds, cv::IMREAD_COLOR);
    if(im.empty())
    {
        log_e("vstream client fail to decode received img");
        return false;
    }
    //----
    Sp<Img> p = mkSp<ImgCv>(im);
    onImg(p);
    if(p_fcb!=nullptr)
        p_fcb(p);

    return true;
}