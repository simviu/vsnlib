#include "vsn/ocv_hlpr.h"

using namespace ocv;

//--------
VideoCv::VideoCv(CStr& s)
{
    cap_.open(s);

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
