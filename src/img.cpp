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
    
    auto ls = p.axis(l);
    Color rgb[3]{{255,0,0}, {0,255,0}, {0,0,255}};
    int i=0;
    for(int i=0;i<3;i++)
    {
        Line2d l = cc.proj(ls[i]);
        draw({l}, rgb[i], w);
    }
}

//-----
void Img::draw(const CamCfg& cc, const Cylinder& cl, const Color& c, float w)
{
   
    auto lns = cl.edges();

   
}
