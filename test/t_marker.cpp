/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
 */

#include "vsn/vsnTest.h"

using namespace vsn;
using namespace ut;
namespace test
{
    //--------------------------
    bool TestMarker::run()
    {
        bool ok = true;
        log_i("-- run test marker...");
        string s = "-- test marker:";
        s += ok?"pass":"fail";
        log_i(s);
        return true;
    }

}