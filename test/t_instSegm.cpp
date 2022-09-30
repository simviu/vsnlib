/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/vsnLibCv.h"
using namespace vsn;
using namespace ut;
using namespace test;

namespace{
    const struct{
        string sf_img = "img_td/1.jpg";
        string sfw_imo = "instSegm.jpg";
        bool enShow = true;
    }lc_;
    //----
}
//--------------------------
bool TestInst::run()
{
    auto p = vsn::Img::loadFile(lc_.sf_img);
    if(p==nullptr)
    {
        log_ef(lc_.sf_img);
        return false;
    }
    auto& im = *p;
    vsn::InstSegm inst;

    // cfg
    auto& c = inst.cfg_;
    c.en_imo = true;
    c.filter = {{240,240,240},{255,255,255}};
    c.areaTH = 100*60;
    inst.onImg(im);

    //-------
    if(lc_.enShow)
    {
        auto p_imo = inst.data().p_imo;
        p_imo->show("InstSegm");
        vsn::show_loop();
    }
    return true;
}

