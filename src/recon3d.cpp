/*
   Author: Sherman Chen
   Create Time: 2023-01-20
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"

using namespace vsn;
using namespace stereo;

namespace{
    struct LCfg{
        float fps = 30;        
    }; LCfg lc_;
    
}
//----
//-----
bool Recon3d::Frm::Imgs::load(string sPath, int i)
{
    string si = to_string(i);
    pL = Img::loadFile(sPath + "/L/"+si+".png");
    pR = Img::loadFile(sPath + "/R/"+si+".png");
    pC = Img::loadFile(sPath + "/C/"+si+".png");
    pD = Img::loadFile(sPath + "/D/"+si+".png");
    pN = Img::loadFile(sPath + "/N/"+si+".png");
    bool ok = (pL!=nullptr) && (pR!=nullptr) && (pC!=nullptr) &&
              (pD!=nullptr) && (pN!=nullptr);
    if(!ok)
        log_e("not all img loaded OK in path:'"+sPath+"'");
    return ok;
}
//----
bool Recon3d::Frm::load(string sPath, int i)
{
    bool ok = true;
    ok &= imgs.load(sPath, i);
    return ok;
}
//----
void Recon3d::init_cmds()
{
    sHelp_ = "(Recon 3d point cloud from frms)";

    Cmd::add("frms", mkSp<Cmd>("dir=<DIR> (Run frms)",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        string sdir = lookup(kv, "dir"); 
        return run_frms(sdir); 
    }));
}
//----
bool Recon3d::onImg(const Frm& f)
{
    // show color
    f.imgs.pC->show("Color");
    return true;
}
//----
bool Recon3d::run_frms(const string& sPath)
{
    //----
    int i=0;
    while(1)
    {
        i++;
        log_i("frm:"+str(i));
        Frm f;
        if(!f.load(sPath, i))
            break;
        
        //---- call
        onImg(f);

        sys::sleep(1.0/lc_.fps);
    }
    return true;
}