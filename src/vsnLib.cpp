#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"

using namespace vsn;
using namespace ocv;
using namespace cv;

//---- Pose
string Pose::str()const{
    stringstream ss;
    ss << "q:'" << q.w() << ", " << q.x() << ", "
        << q.y() << ", " << q.z() << "', ";
    ss << "t:'" << t <<"'";
    return ss.str();
}
//----
string Line2d::str()const 
{ 
    stringstream ss; 
    ss << p1 <<", " << p2; 
    return ss.str(); 
}
//----
string Line::str()const 
{ 
    stringstream ss; 
    ss << p1 <<", " << p2; 
    return ss.str(); 
}

vector<Line> Pose::axis(double l)const
{
    vector<Line> ls;

    vec3 o = t; 
    vec3 v[3]; 
    v[0] << l,0,0;
    v[1] << 0,l,0;
    v[2] << 0,0,l;
    //---- transform with pose
    mat3 R = mat3(q);
    
    for(int i=0;i<3;i++)
    {
        vec3 vd = R*v[i] + t;
        Line l;
        l.p1 = o;  l.p2 = vd;
        ls.push_back(l);
    }
    return ls;
}


//----- utils
extern int vsn::cv_waitkey(int MS)
{
    return cv::waitKey(MS);
}

extern bool vsn::cv_waitESC(int MS)
{
    return cv::waitKey(MS)==27;
}
