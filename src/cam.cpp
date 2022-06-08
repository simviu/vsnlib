#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"

using namespace vsn;
using namespace cv;
using namespace ut;
//----
//-----------------

bool CamCfg::Data::load(CStr& sf)
{

    FileStorage fs(sf, FileStorage::READ);
    
    if (!fs.isOpened())
    {
        log_ef(sf);
        return false;
    }
    
    Mat K1,D1;
    //---- read yaml
    fs["camera_matrix"] >> K1;
    fs["distortion_coefficients"] >> D1;
    fs["image_width"] >> W;
    fs["image_height"] >> H;
    cv2eigen(K1, K);
    cv2eigen(D1, D);
    //----
    stringstream ss;
    ss << "read yaml : "+sf +" good" << endl;
    ss << "  WxH=" << W << "x" << H << endl;
    ss << "  K=" << K << endl;
    ss << "  D=" << D << endl;
    log_i("Camera cfg loaded:"+sf);
    log_i(ss.str());
    return true;
}


//----------- Note: assume undistorted img
vec2 CamCfg::proj(const vec3& p)const
{
    vec2 px; px << 0,0;
    vec3 ph = data_.K * p;
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
