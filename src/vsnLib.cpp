/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
 */

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
//----
Pose Pose::inv()const
{
    // inv[ R | t] = [R^T | -R^T*t]
    Pose p;
    p.q = q.inverse();
    mat3 RT(p.q);
    p.t = -RT*t;
    return p;
}
//----
Pose Pose::operator *(const Pose& p)const
{
    //  | R1 t1 | x | R2 t2 | = | R1R2  R1t2+t1 | 
    //  | 0   1 |   |  0  1 |   |  0       1    |
    //
    Pose T;

    T.q = q * p.q;
    mat3 R1(q);
    T.t = R1*p.t + t;
    return T;
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
