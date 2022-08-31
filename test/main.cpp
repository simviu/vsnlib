/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"

using namespace vsn;
using namespace ut;
using namespace test;
int main(int argc, char ** argv)
{
    string s(argv[0]);
    log_i("--- run : "+s);
    log_i("cur_dir:"+sys::pwd());
    bool ok = true;
    
  //TestMarker  t; ok &= t.run();
  // TestFeature t; ok &= t.run();
  // TestKittiStereo t; ok &= t.run();
    TestBlob t; ok &= t.run();
    if(ok) log_i("All test PASS!");
    else log_e("Failed");
    return ok?0:1;
}
