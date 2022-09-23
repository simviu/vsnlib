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
bool ImgCv::save(ut::CStr& s)const
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
void ImgCv::draw(CStr& s, 
    const Px& px,
    const Color& c,
    float font_scl)
{
    cv::Scalar c1 = toCv(c);
    cv::putText(im_,s,{px.x,px.y},cv::FONT_HERSHEY_DUPLEX,
        font_scl ,c1, 2, false);
}

//------
void ImgCv::draw(const vec2s& vs, const Color& c, float w)
{
    // note: line width about 1/3 of rectangle size
    for(auto& v : vs)
        draw(ut::Rect(toPx(v), Sz(w,w)), c, w*0.3);
}

//----
void ImgCv::draw(const vector<Line2d>& lns, const Color& c, double w)
{
    for(auto& l : lns)
    {
        Point p1(l.p1.x(), l.p1.y());
        Point p2(l.p2.x(), l.p2.y());
        cv::line(im_, p1, p2, toCv(c), w);
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
void ImgCv::draw(const vector<Circle>& cs, const Color& c, float w)
{
    for(auto& o : cs)
    {
        Point2f cn(o.c.x(), o.c.y()) ;
        cv::circle(im_, cn, o.r, toCv(c), w);
    }
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
void ImgCv::filter(const HSV& c0,
                   const HSV& c1)
{
    cv::inRange(im_, toCv(c0), toCv(c1), im_);
}
//-----
Sp<Img> ImgCv::crop(const ut::Rect& r)const
{
    Px c = r.cntr;
    int w = r.sz.w;
    int h = r.sz.h;
    Sz sz = size();
    Rng rr(0, sz.h);
    Rng rc(0, sz.w);
    //
    int r0 = c.y - h/2;
    int r1 = c.y + h/2;
    int c0 = c.x - w/2;
    int c1 = c.x + w/2;
    bool bVal =  rr.isIn(r0) && rr.isIn(r1) &&
                 rc.isIn(c0) && rc.isIn(c1); 
    if(!bVal)
        return nullptr;
    //----
    Range rrow(r0, r1);
    Range rcol(c0, c1);
    Mat imc = im_(rrow, rcol);
    return mkSp<ImgCv>(imc);
}


//-----
vector<Line2d> ImgCv::det(const HoughLnCfg& c)const
{
    // TODO: detect already gray scale
    cv::Mat im = im_;
    if(im.channels()==3)
        cv::cvtColor(im_, im,cv::COLOR_BGR2GRAY);
    if(c.doCanny)
        cv::Canny(im, im, 50, 200, 3); // TODO: cfg
    vector<Vec4i> lines;
    cv::HoughLinesP( im, lines, c.rho, c.theta, c.TH, 
                        c.minLnLen, c.maxLnGap );
    vector<Line2d> lns;
    for(auto& v : lines)
    {
        vec2 p0,p1;
        p0 << v[0], v[1];
        p1 << v[2], v[3];
        Line2d l(p0, p1);
        lns.push_back(l);
    }
    return lns;

}
//----- set/get
vector<Circle> ImgCv::det(const HoughCirCfg& c)const
{
    vector<Vec3f> cs;
    HoughCircles(im_, cs, HOUGH_GRADIENT,
                 c.dp, c.minDist, c.param1, c.param2,
                 c.minRadius, c.maxRadius ); 
    //----
    vector<Circle> cirs;
    for(auto& p : cs)
    {
        vec2 t; t << p[0], p[1]; 
        double r = p[2];
        Circle o{t, r};
        cirs.push_back(o);
    }
    //---
    return cirs;
}

//----- set/get
void ImgCv::set(const Px& px, const Color& c) 
{
    if(!size().isIn(px)) return;
    im_.ptr<BGR>(px.y)[px.x] = BGR(c);
}
bool ImgCv::get(const Px& px, Color& c)const
{
    if(!size().isIn(px)) return false;
    auto& bgr = im_.ptr<const BGR>(px.y)[px.x];
    c = bgr.toUt();
    return true;
}
//---- TODO: template
void ImgCv::set(const Px& px, const HSV& c) 
{
    if(!size().isIn(px)) return;
    im_.ptr<HSV>(px.y)[px.x] = HSV(c);
}
bool ImgCv::get(const Px& px, HSV& c)const
{
    if(!size().isIn(px)) return false;
    c = im_.ptr<const HSV>(px.y)[px.x];
    return true;
}
