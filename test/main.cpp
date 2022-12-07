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
        {"marker"   , mkSp<TestMarker>()}, 
        {"feature"  , mkSp<TestFeature>()}, 
        {"stereo"   , mkSp<TestStereo>()}, 
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
    string s_app(argv[0]);
    log_i("--- run : "+s_app);
    log_i("cur_dir:"+sys::pwd());
    bool ok = true;
    //--- add tests
    Test alltest;
    alltest.tests_ = tests_;

    //----
    stringstream s;
    string sTests = alltest.getTestsStr();
    if(argc<2)
    {
        log_e("test name not provided");
        s << "Usage: "<< s_app <<" test_name|all" << endl;
        s << "  Test list:" << sTests << endl;
        log_i(s.str()); 
        return 1;
    }
    string st = argv[1];
    if(st=="all")
        ok = alltest.run();
    else
        ok = alltest.run(st);
    
    return ok?0:1;
}
