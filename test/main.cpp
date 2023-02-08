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
//-------
namespace{
    //---- test list
    map<string, Sp<Test>> tests_
    {
        {"math"     , mkSp<TestMath>()}, 
        {"marker"   , mkSp<TestMarker>()}, 
        {"feature"  , mkSp<TestFeature>()}, 
        {"inst"     , mkSp<TestInst>()}, 
        {"points"   , mkSp<TestPoints>()},
        {"server"   , mkSp<TestSocketSrvr>()},
        {"client"   , mkSp<TestSocketClnt>()}
    };
}
//-------
// main
//-------
int main(int argc, char ** argv)
{
   
    Test t;
    t.tests_ = tests_;
    return t.run(argc, argv);
}
