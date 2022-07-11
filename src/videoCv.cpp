#include "vsn/ocv_hlpr.h"

using namespace ocv;

//--------
VideoCv::VideoCv(CStr& s)
{
    cap_.open(s, CAP_FFMPEG);
    bool ok = cap_.isOpened();
    int dbg=0;

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
bool VideoCv::createWr(CStr& sf, float fps, const Sz& sz)
{
    p_vwr = mkSp<VideoWriter>(sf, 
        cv::VideoWriter::fourcc('M','J','P','G'), 
        fps, Size(sz.w,sz.h));
}

//-----        
Sp<Video> Video::create(CStr& sf, float fps, const Sz& sz)
{
    auto p = mkSp<VideoCv>();
    p->createWr(sf, fps, sz);
    return p;
}
//----- 
bool VideoCv::write(const Img& im)
{ 
    auto& imc = reinterpret_cast<const ImgCv&>(im);
    if(p_vwr==nullptr)
        return false;
    p_vwr->write(imc.im_);
    return true;
}
