/*
   Author: Sherman Chen
   Create Time: 2022-05-16
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"

using namespace vsn;
using namespace cv;
using namespace ocv;
using namespace ut;
//----
//-----------------
// CamCfg::Data
//-----------------
bool CamCfg::load(CStr& sf)
{

    if(!fexist(sf)) {   
        log_ef("CamCfg '"+ sf+"' not found"); 
        return false; }
    //----
    Mat K1,vD1;

    try{
        FileStorage fs(sf, FileStorage::READ);
        
        if (!fs.isOpened())
        {
            log_ef(sf);
            return false;
        }
        
        //---- read yaml
        fs["camera_matrix"] >> K1;
        fs["distortion_coefficients"] >> vD1;
        fs["image_width"] >> sz.w;
        fs["image_height"] >> sz.h;
    }
    catch(exception& e)
    {
        log_e(string(e.what()));
        return false;
    }
    //------
    cv2eigen(K1, K);
    vec5 vD;
    cv2eigen(vD1, vD);
    D = Dist(vD);
    //----
    stringstream ss;
    ss << "read yaml : "+sf +" good" << endl;
    ss << "  WxH=" << sz.w << "x" << sz.h << endl;
    ss << "  K=" << K << endl;
    ss << "  D=" << D << endl;
    log_i("Camera cfg loaded:"+sf);
    log_i(ss.str());
    Lense l;
    bool ok = toLense(l);
    log_i("Lense data: ");
    log_i(l.str());
    log_i("Distortion:"+D.str());
    return ok; 
}



//--- Note: assume undistorted img
// TODO: Handle cfg
vec2 CamCfg::proj(const vec3& p)const
{
    vec2 px; px << 0,0;
    vec3 ph = K * p;
    if(ph.z()==0) 
        return px;
    // de-homogns;
    px.x() = ph.x()/ ph.z();
    px.y() = ph.y()/ ph.z();

    
    return px;
}

//-----
Line2d CamCfg::proj(const Line& l)const
{
    Line2d ll;
    ll.p1 = proj(l.p1);
    ll.p2 = proj(l.p2);
    return ll;
}
//----
vec3 CamCfg::proj(const vec2& p)const
{
    
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    double x = (p.x() - cx)/fx;
    double y = (p.y() - cy)/fy;
    vec3 v; v << x, y, 1.0;
    return v;
}

//-----
// ref : https://blog.csdn.net/jonathanzh/article/details/104418758
void CamCfg::undis(const vec2s& vds, vec2s& vs)const
{
    vector<Point2f> cds;
    vector<Point2f> cs;
    stringstream s;
 //   s << "vds:" << endl;
    for(auto& v : vds)
    {
    //    s << v << endl; 
        cds.push_back(toCv(v));
    }
    cv::Mat Kc,Dc; 
    eigen2cv(K, Kc);
    eigen2cv(D.V(), Dc);
    cv::undistortPoints(cds, cs, Kc, Dc);
 //   s << "vs:" << endl;
    for(auto& c : cs) 
    {
        vec2 u = toVec(c);
        vec3 vn; vn << u.x(), u.y(), 1;
        vec3 q = K*vn;
        vec2 v; v << q.x(), q.y(); 
    //    s << v << endl;
        vs.push_back(v);
    }
  //  log_d(s.str());
}

//----------
// CamCfg::TDist
//----------
string CamCfg::Dist::str()const
{
    stringstream s;
    s << "k1=" << k1 << ", ";
    s << "k2=" << k2 << ", ";
    s << "k3=" << k3 << ", ";
    s << "p1=" << p1 << ", ";
    s << "p2=" << p2 << ", ";
    return s.str();
}

//----------
// CamCfg::TLense
//----------
string CamCfg::Lense::str()const
{
    stringstream s;
    s << "fx=" << fx << ", fy=" << fy << ", ";
    s << "cx=" << cx << ", cy=" << cy << ", ";
    s << "fovh=" << fovh << ", fovv=" << fovv << ", ";
    s << "fov=" << fov << endl;
    return s.str();
}

//------
bool CamCfg::toLense(Lense& l)const
{
    auto& fx = l.fx; auto& fy = l.fy;
    auto& cx = l.cx; auto& cy = l.cy;
    
    fx = K(0,0);
    fy = K(1,1);
    cx = K(0,2);
    cy = K(1,2);
    if(fx==0 || fy==0)
    {
        log_e("incorrect camCfg");
        return false;
    }

    vec2 q; q << sz.w, sz.h;
    stringstream s;
    //--- proj corner point on
    //  unit focal length 1.0
    vec2 p; p<< (q.x() - cx)/fx, (q.y() - cy)/fy;
    l.fovh = toDgr(2*atan(p.x()));
    l.fovv = toDgr(2*atan(p.y()));
    l.fov  = toDgr(2*atan(p.norm()));
    s << "q=" << q << ", ";
    s << "p=" << p << endl;
    s << "  (Note): only for none distort camCfg" << endl;
    log_d(s.str());
    return true;
}
