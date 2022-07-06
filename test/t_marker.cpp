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
    namespace{
        const struct{
            string sf_camc = "vsnd/picam.yml";
        }l_cfg;
    }
    //--------------------------
    bool TestMarker::run()
    {
        auto& K = l_cfg;
        bool ok = true;
        log_i("-- run test marker...");
        //-----
        CamCfg cc;
        ok &= cc.load(K.sf_camc);

        //-----
        string s = "-- test marker:";
        s += ok?"pass":"fail";
        log_i(s);
        return true;
    }

}