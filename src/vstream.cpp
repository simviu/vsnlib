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
    //----  check jpeg buf
    bool chk_jpg(const Buf& buf)
    {

        if(buf.n<5){
            log_e("chk_jpg(): very small buf, not JPG data");
            return false;
        }
        //---- header 0xffd8
        auto d = buf.p;
        if((d[0]!=0xff) || (d[1]!=0xd8))
        {
            log_e("chk_jpg(): jpg stream prefix 0xffd8 not found");
            return false;
        }
        //---- get image size
        /*
        int w = ( d[3]<<8 )+(d[2]);
        int h = ( d[5]<<8 )+(d[4]);
        stringstream s;
        s << "  jpeg header (w,h)=(" 
            << w << "," << h << ")";
        log_d(s.str());
        */
        return true;
    }
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

    //---- open cam
    bool ok = true;
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
        log_e("  missing [cam=|img=|video=]");
        return false;
    }
    //----
    if(kvs.has("--show"))
        cfg_.en_show = true;
    
    //----
    log_i("vstream server init ok on port:"+to_string(port)+"...");
 //   while(1)
   //     sys::sleep(0.02);

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
    
    if(!svr_.isConnected())
        return;

    cv::Mat im = img2cv(*p);
    vector<uchar> buf;
    cv::imencode(".jpg", im, buf);

    //---- to binary
    auto pb = (uint8_t*)(&buf[0]);
    int n  = buf.size();
    Buf b(pb, n);
    //--- dbg , check jpg buf
    if(!chk_jpg(b)) return;

    //---- send header
    stringstream s;
    s << "dtype=vstream.image ";
    s << "buf_len=" << n << "\n";
    log_d("  send header '"+s.str()+"'");
    svr_.send(s.str());

    //---- send data
    svr_.send(b);
    auto& fi = data_.frm_idx;
    log_d("  sent img "+to_string(fi++));
}
//----
bool Client::connect(const string& sHost, int port)
{
    log_i("vstream Client init...");
    if(!clnt_.connect(sHost, port))
    {
        log_e("  vstream client fail to connect to server "+
                    sHost + ":"+ to_string(port));
        return false;
    }    

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
        if(!clnt_.isConnected()) 
            break;

        run_once();
        sys::sleep(lc_.t_loop_delay);
    }
    log_i("vstream client disconnected");
}

//----
bool Client::run_once()
{
    //---- get header string
    string sln;
    if(!clnt_.recvLn(sln))
    {
        log_e("failed to get vstream header");
        return false;
    }
    log_d("  recv header '"+sln+"'");
    //---- chk
    KeyVals kvs(sln);
    string st = kvs.get("dtype");
    if(st!="vstream.image")
    {
        log_e("expect header 'dtype=vstream.image'");
        return false;
    }
    int len=0; 
    if(!kvs.get("buf_len", len)) 
        return false;
    log_d("  got buf_len:"+to_string(len));
    //-----
    Buf buf(len);
    if(!clnt_.recv(buf))
        return false;
    //----
    //----
    if(!chk_jpg(buf))
        return false;
    //----
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
    auto& fi = data_.frm_idx;
    stringstream s;
    //----
    Sp<Img> p = mkSp<ImgCv>(im);
    s << "  recv img " << fi++ << ", size:" << p->size().str();
    log_d(s.str());

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
    bool ok = connect(s_host, port); 
    return ok;
}
