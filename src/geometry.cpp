/*
   Author: Sherman Chen
   Create Time: 2022-09-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/vsnLibCv.h"
#include "vsn/pcl_utils.h"
#include "json/json.h"


using namespace vsn;

//----- projection fucntions
vec3 Plane::proj(const vec3& p)
{
    vec3 pc = p-c;
    float l = n.norm(); // dbg
    double d = pc.dot(n);
    vec3 dv = d*n;
    vec3 pp = p - dv;
    return pp;
}
bool Plane::cross(const Line& l, vec3& p)
{
    vec3 nl = l.nv();
    if(nl == n) return false; // parellel
    vec3 p1 = l.p1;
    vec3 pp = proj(p1);
    double d = (pp - p1).norm();
    // nl proj on vertial n, causing scale.
    double s = 1.0/nl.dot(-n); 
    p = p1 + nl*(d*s);
    return true;
}

//-------
vec3s Cube::points()const
{
    // +x -x  -x +x
    // +y +y  -y -y
    // 0,  1, 2, 3   +z
    // 4,  5, 6, 7   -z
    vector<vector<double>> dss{
        {1,1,1},{-1,1,1},{-1,-1,1},{1,-1,1},
        {1,1,-1},{-1,1,-1},{-1,-1,-1},{1,-1,-1}
    };
    vec3s vs;
    for(auto& ds : dss)
    {
        vec3 v; v << 
            ds[0]*sz_.x()*0.5, 
            ds[1]*sz_.y()*0.5, 
            ds[2]*sz_.z()*0.5;
        vs.push_back(pose * v);
    }
    return vs;
}
//-------
vector<Line> Cube::edges()const
{
    auto vs = points();
    vector<Line> lns;
    for(int i=0;i<4;i++)
    {
        int in = (i==3)?0:i+1; // next 1,2,3,0
        lns.push_back(Line(vs[i], vs[in])); // top
        lns.push_back(Line(vs[i], vs[i+4])); // vertical
        lns.push_back(Line(vs[i+4], vs[in+4])); // bottom
    }

    return lns;

}

//-------
vec3s Cylinder::points()const
{
    vec3s vs;
    if(N_fan==0)return vs;
    double L = l();
    for(int i=0;i<N_fan;i++)
    {
        float a = M_PI*2*i/N_fan;
        vec3 p0; p0 << cos(a),sin(a), L/2;
        vec3 p1=p0; p1.z() = -L/2;
        vs.push_back(pose * p0);
        vs.push_back(pose * p1);
    }
    return vs;
}
//-------
vector<Line> Cylinder::edges()const
{
    auto vs = points();
    assert(vs.size()==N_fan*2);    
    vector<Line> lns;
    for(int i=0;i<N_fan;i++)
    {
        int in = i+1;
        if(in==N_fan) in = 0;
        lns.push_back(Line(vs[2*i], vs[2*in])); // top 
        lns.push_back(Line(vs[2*i], vs[2*i+1])); // vertical
        lns.push_back(Line(vs[2*i+1], vs[2*in+1])); // bottom

    }
    return lns;
}
