/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
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
    ss << "("<< egn::str(p1) <<")->(" 
        << egn::str(p2) << ")"; 
    return ss.str(); 
}
//----
string Line::str()const 
{ 
    stringstream ss; 
    ss << "(" << egn::str(p1) 
        <<")->(" << egn::str(p2) << ")"; 
    return ss.str(); 
}
void Line::operator *= (const Pose& P)
{
    p1 = P.q*p1 + P.t;
    p2 = P.q*p2 + P.t;
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
//----
vec3 Pose::operator *(const vec3& v)const
{
    vec3 vr;
    mat3 R(q);
    vr = R*v + t;
    return vr;
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
extern void vsn::show_loop()
{
    while(!cv_waitESC(10));
}
