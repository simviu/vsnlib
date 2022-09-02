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
        string sf_test = "points.pcd";
    }lc_;
}
//--------------------------
bool TestPoints::test_basic()
{
    return true;
}

//--------------------------
bool TestPoints::run()
{
    //--- test pcl orign tests
    //test_pcl_wr();
    //test_pcl_vis();
    //---- test basic
    bool ok = true;
    Points pd;
    pd.gen_cylinder();
    ok &= pd.save(lc_.sf_test);
    Points pdr;
    ok &= pdr.load(lc_.sf_test);

    auto p_vis = Points::Vis::create();
    auto& vis = *p_vis;
    vis.add(pdr, "test");
    while (vis.spin())
        std::this_thread::sleep_for(100ms);

    return ok;
}


