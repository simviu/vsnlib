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
VideoCv::VideoCv(int idx, Sz res)
{
    log_i("Open camera :"+to_string(idx));
    cap_.open(idx);
    if(!cap_.isOpened())
        return;

    //---- try set resolution
    bool ok = true;
    ok &= cap_.set(CAP_PROP_FRAME_WIDTH,  res.w);
    ok &= cap_.set(CAP_PROP_FRAME_HEIGHT, res.h);
    if(!ok)
    {
        log_e("  Failed to init resolution sz:" + res.str());
        return;
    }

    //----
    init();
}

//--------
VideoCv::VideoCv(CStr& s)
{
    cap_.open(s, CAP_FFMPEG);
    if(!cap_.isOpened())
        return;

    init();
}

//----
void VideoCv::init()
{
    //-- get properties
    cfg_.sz.w = cap_.get(CAP_PROP_FRAME_WIDTH);
    cfg_.sz.h = cap_.get(CAP_PROP_FRAME_HEIGHT);
    cfg_.fps = cap_.get(CAP_PROP_FPS);

    //----
    stringstream ss;
    ss << "  Open video OK" << endl;
    auto& c = cfg_;
    ss << "  size:" << c.sz.w << "x" << c.sz.h << ", ";
    ss << "  fps:" << c.fps << endl;

    log_i(ss.str());
}

//--------
Sp<Video> Video::open(int i, Sz sz)
{

    auto p = mkSp<VideoCv>(i, sz);
    if(p->isOpen()) 
        return p;
    log_e("Failed to open camera:"+to_string(i));
    return nullptr;

}
//--------
Sp<Video> Video::open(CStr& s)
{

    auto p = mkSp<VideoCv>(s);
    if(p->isOpen()) 
        return p;
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
bool VideoCv::createWr(CStr& sf)
{
    Sz sz = cfg_.sz;
    p_vwr = mkSp<VideoWriter>(sf, 
        cv::VideoWriter::fourcc('M','J','P','G'), 
    //  cv::VideoWriter::fourcc('H','2','6','4'), 
    //  cv::VideoWriter::fourcc('m','p','4','v'), 
    //  cv::VideoWriter::fourcc('P','I','M','1'), 
        cfg_.fps, Size(sz.w,sz.h));
    return p_vwr->isOpened();
}

//-----  static
Sp<Video> Video::create(CStr& sf, const Cfg& cfg)
{
    log_i("create video:'"+sf+"'...");

    auto p = mkSp<VideoCv>();
    p->cfg_ = cfg;
    bool ok = p->createWr(sf);
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
    //p_vwr->write(imc.im_);
    auto& vw = *p_vwr;
    Mat imw; imc.im_.copyTo(imw);
    vw.write(imw);
    return true;
}
//-----
void VideoCv::close()
{
   // if(p_vwr==nullptr)
     //   p_vwr->release();
}

