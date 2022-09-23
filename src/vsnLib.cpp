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

//-----------
// Eulter
//-----------
// Use opencv camera coordinate order:
//   z forward, y down, x right
//   Order: (y):yaw, (x):pitch, (z):roll 
//   Unit degree.
// TODO: not yet
/*
Euler::Euler(const quat& q)
{
    //---- based on Eigen doc,
    // 0,1,2 --> x, y, z
    // mat is angleAxis multiple of define order.
    // in our case ypr -> yxz -> 1,0,2
    auto e = q.matrix().eulerAngles(1,0,2);
    // TODO: Eigen eulerAngles got only Z-Y-Z
    y = toDgr(e.y());
    p = toDgr(e.x());
    r = toDgr(e.z());
}
*/
//-----
bool Euler::parse(const string& s)
{
    vector<double> ds;
    if(!s2data(s, ds)) return false;
    if(ds.size()<3) return false;
    y = ds[0];
    p = ds[1];
    r = ds[2];
    return true;
}

//-----
quat Euler::q()const
{
    vec3 nx,ny,nz;
    nx << 1,0,0; ny << 0,1,0; nz << 0,0,1;
    mat3 my = rotmat(ny, toRad(r));
    mat3 mp = rotmat(nx, toRad(p));
    mat3 mr = rotmat(nz, toRad(y));
    quat q(my * mp * mr);
    return q;
}

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

void Line::trans(const Pose& P)
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
