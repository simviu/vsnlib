/*
   Author: Sherman Chen
   Create Time: 2022-05-17
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/ocv_hlpr.h"


using namespace ocv;
using namespace vsn;

namespace{
            


}
//---------
void ImgCv::rot(double dgr)
{
    cv::Point2f center((im_.cols - 1)/2.0, (im_.rows - 1)/2.0);
    cv::Mat R = cv::getRotationMatrix2D( center,dgr , 1.0 );
    cv::Mat imr;
    cv::warpAffine(im_, imr, R, im_.size());
    im_ = imr;
}

//---------

void ImgCv::undistort(const CamCfg& cc)
{
    cv::Mat Kc, Dc;
    eigen2cv(cc.K, Kc);
    eigen2cv(cc.D.V(), Dc);    
    Mat imd;
    cv::undistort(im_, imd, Kc, Dc);
    im_ = imd;
}


//-----------------
bool ImgCv::load(ut::CStr& s)
{
    im_ = cv::imread(s);
    bool ok = val();
    if(ok)
        log::inf("Img load:"+s);
    else
        log::errf(s);
    return ok;
}
//-----------------
bool ImgCv::save(ut::CStr& s)
{
    log_i("Save img to:"+s+"...");
    bool ok = cv::imwrite(s, im_);
    if(ok)
        log::inf("Img save:"+s);
    else 
        log::errf(s);
    return ok;
}

   
//-----
void ImgCv::show(CStr& sWind)
{
    cv::imshow(sWind, im_);
    cv::waitKey(1);
}
//----
void ImgCv::text(CStr& s, 
    const Px& px,
    const Color& c)
{
    cv::Scalar c1 = toCv(c);
    cv::putText(im_,s,{px.x,px.y},cv::FONT_HERSHEY_DUPLEX,
        1 ,c1, 2, false);
}
//----
void ImgCv::line(const Line2d& l, const Color& c, double w)
{
    Point p1(l.p1.x(), l.p1.y());
    Point p2(l.p2.x(), l.p2.y());
    cv::line(im_, p1, p2, toCv(c), w);
}
//-----
void ImgCv::axis(const CamCfg& cc, 
    const Pose& p, double l, double w)
{
    
    auto ls = p.axis(l);
    Color rgb[3]{{255,0,0}, {0,255,0}, {0,0,255}};
    int i=0;
    for(int i=0;i<3;i++)
    {
        Line2d ll = cc.proj(ls[i]);
        line(ll, rgb[i], w);
    }
}

