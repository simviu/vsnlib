/*
   Author: Sherman Chen
   Create Time: 2022-05-16
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"
using namespace vsn;
using namespace ut;


//----factory implementation use ImgCv;
Sp<Img> Img::create()
{
    return make_shared<ocv::ImgCv>();
}

