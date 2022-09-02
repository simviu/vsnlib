/*
   Author: Sherman Chen
   Create Time: 2022-09-01
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/vsnLibCv.h"
#include "vsn/pcl_utils.h"

using namespace vsn;
using namespace ut;
using namespace test;

namespace{
    const struct{
    }lc_;
}
//--------------------------
bool TestPoints::run()
{
    // test basic
    //test_pcl_wr();
    test_pcl_vis();
    
    return true;
}


