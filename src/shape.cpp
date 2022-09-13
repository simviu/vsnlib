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
            ds[0]*sz.x(), ds[1]*sz.y(), ds[2]*sz.z();
        v += c;   
        vs.push_back(v);
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
void Cylinder::gen(vec3s& vs0, vec3s& vs1)const
{
    if(N_fan==0)return;
    for(int i=0;i<N_fan;i++)
    {
        float a = M_PI*2*i/N_fan;
        vec2 pr; pr << cos(a),sin(a);
        pr *= r;
        vec3 p0; p0 << pr, l;
        vec3 p1; p1 << pr, -l;
        vec3 p0w = pose * p0;
        vec3 p1w = pose * p1;
        vs0.push_back(p0w);
        vs1.push_back(p1w);
    }
}
