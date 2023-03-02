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
void Server::init_cmds()
{
    sHelp_ = "(vstream server commands)";

    //----
    add("init", mkSp<Cmd>("port=<PORT> [cam=ID | img=<FILE> | video=<FILE>] [--show]",
    [&](CStrs& args)->bool{  return init(args); }));
}
//---
bool Server::init(CStrs& args)
{
    KeyVals kvs(args);
    int port=-1; 
    if(!kvs.get("port", port)) 
        return false;
    if(!init(port)) return false;
    //---
    bool ok = true;
    string si;  
    if(!kvs.get("src", si)) 
        return false;

    //---- open cam
    int cam_id=0;
    if(kvs.has("cam"))
    {
        ok &= kvs.get("cam", cam_id);        
        ok &= open(cam_id);
    }
    else if(kvs.has("img"))
        ok &= openImg(kvs["img"]);
    else if(kvs.has("video"))
        ok &= open(kvs["video"]);
    else{
        log_e("  missing src [cam|img|video]");
        return false;
    }
    //----
    if(kvs.has("--show"))
        cfg_.en_show = true;
    //----
    log_i("vstream server init ok on port:"+to_string(port));
    return ok;
        
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
bool Server::openImg(const string& sf)
{
    p_img_  = Img::loadFile(sf);
    return (p_img_!=nullptr);
    
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
    Sp<Img> p = nullptr;
    if(p_video_!=nullptr)
        p = p_video_->read();
    else if(p_img_!=nullptr)
        p = p_img_;
    
    //----
    if(p!=nullptr)
        send(p);
    if(cfg_.en_show)
        p->show("vstream Server");

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
    

    //--- start loop thread
    thd_ = std::thread([&](){
        run_loop();
    });
    thd_.detach();

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

    //---
    if(cfg_.en_show)
        p->show("vstream client");

    return true;
}


//----
void Client::init_cmds()
{
    sHelp_ = "(vstream client commands)";

    //----
    add("connect", mkSp<Cmd>("host=<HOST> port=<PORT> [--show]",
    [&](CStrs& args)->bool{  
        return connect(args);        
    }));
}
//-----
bool Client::connect(CStrs& args)
{
    KeyVals kvs(args);
    string s_host;
    int port = -1; 
    if(!kvs.get("host", s_host)) 
        return false;
    if(!kvs.get("port", port)) 
        return false;
    //----
    if(kvs.has("--show"))
        cfg_.en_show = true;
    //----
    return connect(s_host, port); 
}
