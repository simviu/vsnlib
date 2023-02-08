/*
   Author: Sherman Chen
   Create Time: 2023-02-08
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
//----
bool TestMath::t_dgrIn180()
{
    log_i("  TestMath::t_dgrIn180()...");
    double ds[]{-400, -370, -190, -150, -91, -45, 0, 45, 92, 150, 380, 400};
    for(auto& d : ds)
    {
        double r = dgrIn180(d);
        log_i(to_string(d)+" => "+to_string(r));
    }
    return true;
}

//----
bool TestMath::run()
{
    log_i("TestMath::run()...");
    bool ok = true;
    ok &= t_dgrIn180();
    return true;
}
