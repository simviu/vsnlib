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

