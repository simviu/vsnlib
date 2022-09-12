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
//----
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
