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


bool HSV::set(const string& str)
{
    vector<int> ds;
    if(!s2data(str, ds)) return false;
    if(ds.size()<3) return false;
    h = ds[0];
    s = ds[1];
    v = ds[2];

    return true;
}

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
 
bool Euler::set(const string& s)
{
    vector<double> ds;
    if(!s2data(s, ds)) return false;
    if(ds.size()<3) return false;
    rx = ds[0];
    ry = ds[1];
    rz = ds[2];
    return true;
}



//---- Pose
string Pose::str()const{
    stringstream ss;
    ss << "q:'" << q.w() << ", " << q.x() << ", "
        << q.y() << ", " << q.z() << "', ";
    ss << "t:'" << t <<"'";
    return ss.str();
}
//-----
Pose Pose::avg(const vector<Pose>& ps)
{
    Pose a;
    int i=0;
    for(const auto& pi : ps)
    {

        if(i==0)
        {
            a.q = pi.q;
            i++;
            continue;
        }
        //---- average quat by slerp()
        double t = 1.0/(i+1.0); 
        a.q = a.q.slerp(t, pi.q);
        i++;
    }
    //---- average t
    int N = ps.size();
    vec3 t; t << 0,0,0;
    for(auto& pi : ps)
        t += pi.t;
    a.t = t*(1.0/N);
    return a;
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

//----------------------------
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


//-------
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
vector<Line2d> RRect2d::lines()const
{
    vector<Line2d> lns;
    double dx = sz.x()/2;
    double dy = sz.y()/2;
    vec2 p0; p0 << -dx,  dy;
    vec2 p1; p1 <<  dx,  dy;
    vec2 p2; p2 <<  dx, -dy;
    vec2 p3; p3 << -dx, -dy;
    vec2s ps{p0, p1, p2, p3};
    mat2 R = rotmat(a);
    //----
    for(auto& p : ps)
        p = (R*p) + c;
    //----
    for(int i=0;i<4;i++)
        lns.push_back(
            Line2d(ps[i], ps[(i==3)?0:i+1]));
    
    return lns;
}

//----
/*
Line::Crossr Line::operator ^(const Line& l)
{
    Crossr r;
    return r;
}
*/
//----
vec3 Ray::operator ^(const vec3& p)const
{
    double t = n.dot(p - o);
    return o + n*t;
}

//----
Ray::Xd Ray::operator ^(const Ray& r)const
{
    Xd xd;
    auto& p1 = o;
    auto& p2 = r.o;
    auto& n1 = n;
    auto& n2 = r.n;
    vec3 x12 = n1.cross(n2);
    double d12 = x12.squaredNorm(); 
    // parallel :
    //   calc center of p1,p2
    //   proj onto ray1, ray2
    if(d12==0) 
    {
        xd.bPar = true;
        vec3 c = (p1+p2)*0.5;
        vec3 a = (*this)^c;
        vec3 b = r^c;
        xd.l = Line(a, b);
        return xd;
    }
    //----
    double t1 = ((p2-p1).cross(n2)).dot(x12) / d12; 
    double t2 = ((p2-p1).cross(n1)).dot(x12) / d12; 
    vec3 a = p1 + n1*t1;
    vec3 b = p2 + n2*t2;

    //----
    xd.l = Line(a, b);
    xd.t1 = t1;
    xd.t2 = t2;
    xd.bBehind = (t1<0) || (t2<0); 
    xd.bVal = !(xd.bBehind || xd.bPar);
    return xd;
}
//----
void Ray::trans(const Pose& T)
{
    vec3 o1 = T * o;
    o = o1;
    mat3 R(T.q);
    vec3 n1 = R * n;
    n = n1;
    n.normalize();
    
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
extern bool vsn::findImgs(const string& sdir,
                        vector<string>& sfs)
{
    if(!sys::exists(sdir))
    {
        log_e("Dir not exists: '"+sdir+"'");
        return false;
    }
    //----
    cv::glob(sdir, sfs);
    return true;
}
