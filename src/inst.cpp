/*
   Author: Sherman Chen
   Create Time: 2022-08-31
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"

using namespace vsn;
using namespace ut;

//-----------------
// detect
//-----------------
bool Instance::detect(const Img& im)
{
    auto p1 = im.copy();
    auto& im1 = *p1;
    im1.filter({240,240,240},{255,255,255});
    im1.show("im1");
  
    if(cfg_.enShow)
      while(!vsn::cv_waitESC(10));
    return true;
}

