/*
   Author: Sherman Chen
   Create Time: 2022-07-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLibCv.h"

using namespace ocv;

//--------
VideoCv::VideoCv(CStr& s)
{
    cap_.open(s, CAP_FFMPEG);
    bool ok = cap_.isOpened();
    if(!ok) return;
    //-- get properties
    cfg_.sz.w = cap_.get(CAP_PROP_FRAME_WIDTH);
    cfg_.sz.h = cap_.get(CAP_PROP_FRAME_HEIGHT);
    cfg_.fps = cap_.get(CAP_PROP_FPS);
}
//--------
Sp<Video> Video::open(CStr& s)
{
    
    auto p = mkSp<VideoCv>(s);
    if(p->isOpen()) 
    {
        log_i("Open OK video:"+s);
        return p;
    }
    log_e("Failed to open video:"+s);
    return nullptr;
}
//--------
Sp<Img> VideoCv::read()
{
    Mat im;
    cap_.read(im);
    if(im.empty())
        return nullptr;
    auto p = mkSp<ImgCv>(im);
    return p;
}
//-----        
bool VideoCv::createWr(CStr& sf, const Cfg& cfg)
{
    Sz sz = cfg.sz;
    p_vwr = mkSp<VideoWriter>(sf, 
    //  cv::VideoWriter::fourcc('M','J','P','G'), 
        cv::VideoWriter::fourcc('H','2','6','4'), 
        cfg.fps, Size(sz.w,sz.h));
    return p_vwr->isOpened();
}

//-----        
Sp<Video> Video::create(CStr& sf, const Cfg& cfg)
{
    log_i("create video:'"+sf+"'...");

    auto p = mkSp<VideoCv>();
    bool ok = p->createWr(sf, cfg);
    if(!ok)
        log_ef(sf);
    else log_i("create video OK");
    return p;
}
//----- 
bool VideoCv::write(const Img& im)
{ 
    auto& imc = reinterpret_cast<const ImgCv&>(im);
    if(p_vwr==nullptr)
        return false;
    if(!p_vwr->isOpened())
        return false;
    p_vwr->write(imc.im_);
    return true;
}

