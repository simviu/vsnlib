/*
   Author: Sherman Chen
   Create Time: 2023-01-20
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"
#include "json/json.h"
#include "vsn/vsnLibCv.h"

using namespace vsn;
using namespace stereo;

namespace{
    struct LCfg{
        float fps = 30; 
    }; LCfg lc_;
    
}
//----
bool Recon3d::Cfg::load(const string& sf)
{

    log_i("Load Recon3d cfg :'"+sf+"'");
    ifstream ifs(sf);
    sys::FPath fp(sf);

    if(!ifs)
    {
        log_ef(sf);
        return false;
    }
    bool ok = true;
    //----
    try{

        Json::Reader rdr;
        Json::Value jd;
        rdr.parse(ifs, jd);
       
        string sfc = fp.path + jd["cams_cfg"].asString();
        if(!cams.load(sfc)) 
            return false;
        
    }
    catch(exception& e)
    {
        log_e("exception caught:"+string(e.what()));
        return false;
    }
    
    if(!ok)
    {
        log_e("CamsCfg::load() json error");
        return false;
    }    
    log_i("Recon3d cfg loaded '"+sf +"'");
    return true;
}
//-----
bool Recon3d::loadFrm_imgs(Frm& frm, const string& sPath, int i)
{
    string si = to_string(i);
    auto& sDirs = cfg_.frms.sDirs;
    int N = sDirs.size();
    int k=0;
    for(k=0;k<N;k++)
    {
        string sdir = sDirs[k];
        auto p = Img::loadFile(sPath + "/"+sdir+"/"+si+".png");
        if(p==nullptr) break;
        frm.imgs.push_back(p);
    }
    if(k<N)
    {
        log_e("not all img loaded OK in path:'"+sPath+"'");
        return false;
    }
    return true;

}
//----

bool Recon3d::loadFrm(Frm& frm, const string& sPath, int i)
{
    bool ok = true;
    ok &= loadFrm_imgs(frm, sPath, i);
    return ok;
}
//----
void Recon3d::init_cmds()
{
    sHelp_ = "(Recon 3d point cloud from frms)";


    Cmd::add("init", mkSp<Cmd>("cfg=<CFG_FILE>",
    [&](CStrs& args)->bool{ 
        StrTbl kv; parseKV(args, kv);
        return cfg_.load(lookup(kv, "cfg")); 
    }));

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
    if(0)
    {
        int i = cfg_.frms.color_img;
        assert(i<f.imgs.size());
        auto p = f.imgs[i];
        if(p!=nullptr)
            p->show("Color");
    }
    //---- show undistorted img L
    {
        auto p = f.imgs[0];
        assert(p!=nullptr);
        p->show("Left");
        auto& cc = cfg_.cams.cams[0].camc;
        ImgCv im(*p);
        cv::Mat imuc;
        cv::Mat map1, map2;
        cv::eigen2cv(cc.map1, map1);
        cv::eigen2cv(cc.map2, map2);
        cv::remap(im.im_, imuc, map1, map2, cv::INTER_LINEAR);
        //----
        ImgCv imu(imuc);
        imu.show("Left undist");
    }
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
        if(!loadFrm(f, sPath, i))
            break;
        
        //---- call
        onImg(f);

        sys::sleep(1.0/lc_.fps);
    }
    return true;
}