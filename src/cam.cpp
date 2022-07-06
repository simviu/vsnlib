/*
   Author: Sherman Chen
   Create Time: 2022-05-16
   Email: schen@simviu.com
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

    FileStorage fs(sf, FileStorage::READ);
    
    if (!fs.isOpened())
    {
        log_ef(sf);
        return false;
    }
    
    Mat K1,vD1;
    //---- read yaml
    fs["camera_matrix"] >> K1;
    fs["distortion_coefficients"] >> vD1;
    fs["image_width"] >> sz.w;
    fs["image_height"] >> sz.h;
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
    return ok; 
}



//----------- Note: assume undistorted img
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
//-----
void CamCfg::undis(const vec2s& vds, vec2s& vs)const
{
    vector<Point2f> cds;
    vector<Point2f> cs;
    for(auto& v : vds) cds.push_back(toCv(v));
    cv::Mat Kc,Dc; 
    eigen2cv(K, Kc);
    eigen2cv(D.V(), Dc);
    cv::undistortPoints(cds, cs, Kc, Dc);
    for(auto& c : cs) vs.push_back(toVec(c));
}

//----------
// CamCfg::TDist
//----------
string CamCfg::Dist::str()const
{
    stringstream s;
    cout << "k1=" << k1 << ", ";
    cout << "k2=" << k2 << ", ";
    cout << "k3=" << k3 << ", ";
    cout << "p1=" << p1 << ", ";
    cout << "p2=" << p2 << ", ";
    return s.str();
}

//----------
// CamCfg::TLense
//----------
string CamCfg::Lense::str()const
{
    stringstream s;
    s << "fx=" << fx << ", fy=" << fy << ", ";
    s << "cx=" << fx << ", cy=" << cy << ", ";
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
    //--- undistor corner pnt
    vec2 qd; qd << sz.w, sz.h;
    vec2s qs;
    undis({qd}, qs);
    vec2 q = qs[0];
    //--- proj corner point on
    //  unit focal length 1.0
    vec2 p; p<< (q.x() - cx)/fx, (q.y() - cy)/fy;
    l.fovh = 2*atan(p.x());
    l.fovv = 2*atan(p.y());
    l.fov = 2*atan(p.norm());
    return true;
}
