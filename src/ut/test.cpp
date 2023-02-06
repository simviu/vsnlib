/*
   Author: Sherman Chen
   Create Time: 2023-02-06
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "ut/cutil.h"

using namespace ut;


bool Test::run()
{ 
    if(tests_.size()==0)
    {
        log_e("tests empty");
        return false; 
    }
    //----
    bool ok = true;
    
    for(auto it : tests_)
    {
        string s = it.first;
        ok &= run(s);
    }

    if(!ok)
         log_e("  tests has failure");
    else log_i(" tests passed");
    return ok;
}
//----
bool Test::run(const string& s)
{
    auto it = tests_.find(s);
    if(it==tests_.end())
    {
        log_e("  test not found:"+s);
        return false;
    }
    auto& t = *tests_[s];
    log_i("  ---- run test:'"+s+"'...");
    bool ok = t.run();
    if(ok) log_i("  ---- test '"+s+"' pass");
    else   log_e("  ---- test '"+s+"' fail");
    return ok;
}
//----
string Test::getTestsStr()const
{
    string s;
    for(auto it : tests_)
        s += it.first; s += " ";
    return s;
}

//----
int Test::run(int argc, char ** argv)
{
    string s_app(argv[0]);
    log_i("--- run : "+s_app);
    log_i("cur_dir:"+sys::pwd());
    bool ok = true;


    //----
    stringstream s;
    string sTests = getTestsStr();
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
        ok = run();
    else
        ok = run(st);
    
    return ok?0:1;
}
