/*
   Author: Sherman Chen
   Create Time: 2022-05-17
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/vsnLibCv.h"


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
bool ImgCv::load(ut::CStr& s, int cvFlag)
{
    im_ = cv::imread(s, cvFlag);
    bool ok = val();
    if(ok)
        log_i("Img load:"+s);
    else
        log_ef(s);
    return ok;
}
//-----------------
bool ImgCv::save(ut::CStr& s)
{
    log_i("Save img to:"+s+"...");
    bool ok = cv::imwrite(s, im_);
    if(ok)
        log_i("Img save:"+s);
    else 
        log_ef(s);
    return ok;
}

   
//-----
void ImgCv::show(CStr& sWind)const
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

//--------
void ImgCv::draw(const ut::Rect& r, 
                 const Color& c,  float w)
{
    
    Px p0 = r.p0();
    Px p1 = r.p1();
    cv::rectangle(im_, 
        Point(p0.x, p0.y), 
        Point(p1.x, p1.y), 
        toCv(c), w);
}

//--------
void ImgCv::toGray()
{
    cv::cvtColor(im_, im_,cv::COLOR_BGR2GRAY);
}
//--------
void ImgCv::toHsv()
{
    cv::cvtColor(im_, im_,cv::COLOR_BGR2HSV);
}
//--------
void ImgCv::filter(const Color& c0,
                   const Color& c1)
{
    cv::inRange(im_, toCv(c0), toCv(c1), im_);
}
