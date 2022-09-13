/*
   Author: Sherman Chen
   Create Time: 2022-05-16
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/vsnLibCv.h"
using namespace vsn;
using namespace ut;

namespace{
    //-----
    vector<Line> pose2axis(const Pose& p, double l)
    {
        vector<Line> ls;

        vec3 o = p.t; 
        vec3 v[3]; 
        v[0] << l,0,0;
        v[1] << 0,l,0;
        v[2] << 0,0,l;
        //---- transform with pose
        mat3 R = mat3(p.q);
        
        for(int i=0;i<3;i++)
        {
            vec3 vd = R*v[i] + p.t;
            Line l;
            l.p1 = o;  l.p2 = vd;
            ls.push_back(l);
        }
        return ls;
    }

}

//----factory implementation use ImgCv;
Sp<Img> Img::create()
{
    return make_shared<ocv::ImgCv>();
}

//-----
Sp<Img> Img::loadFile(const string& sf, int cvFlags)
{
    auto p = Img::create();
    if(p->load(sf))
      return p;
    return nullptr;

}


//-----
void Img::draw(const CamCfg& cc, const Axis& a)
{
    auto& p = a.pose;
    auto& l = a.l;
    auto& w = a.w;
    
    auto ls = pose2axis(p, l);
    Color rgb[3]{{255,0,0}, {0,255,0}, {0,0,255}};
    int i=0;
    for(int i=0;i<3;i++)
    {
        Line2d l = cc.proj(ls[i]);
        draw({l}, rgb[i], w);
    }
}
//----
void Img::draw(const CamCfg& cc, const vector<Line>& lns, const Color& c, float w)
{
    for(auto& l : lns)
        draw({cc.proj(l)}, c, w);
}

//-----
/* TODO: replaced by draw lines
void Img::draw(const CamCfg& cc, const Cylinder& cl, const Color& c, float w)
{
    auto lns = cl.edges();
    for(auto& l : lns)
        draw({cc.proj(l)}, c, w);
}
//----
void Img::draw(const CamCfg& cc, const Cube& cb, const Color& c, float w=2)
{
    auto lns = cb.edges();
    for(auto& l : lns)
        draw({cc.proj(l)}, c, w);

}
*/
