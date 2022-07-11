#include "vsn/vsnTool.h"
//#include "vsn/ocv_hlpr.h"

using namespace app;

//----
CmdMarker::CmdMarker():
    Cmd("ArUco marker commands")
{
    //---- 'det'
    {
        string sH = "detect marker, usage: det img=<FILE>";
        add("det", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_det(args); }));
    }
    //---- 'pose'
    {
        string sH = "detect marker and pose estimate \n";
        sH += "Usage:pose img=<FILE> cfg=<FILE_CFG> camc=<FILE_CAM_CFG wdir=<WDIR>\n";
        add("pose", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_pose(args); }));
    }
}
//----
bool CmdMarker::run_det(CStrs& args)const
{
    StrTbl kv;
    parseKV(args, kv);
    string sf = lookup(kv, string("img"));
    auto p = Img::create();
    if(!p->load(sf))
        return false;
    vector<Marker> ms;
    Marker::detect(*p, ms);
    stringstream ss;
    for(auto& m : ms)
        ss << "Found marker:" << m.str() << endl;
    log_i(ss.str());
    return true;
}

//----
bool CmdMarker::run_pose(CStrs& args)
{
    bool ok = true;
    StrTbl kv;
    parseKV(args, kv);
    string sfi = lookup(kv, string("img"));
    string sfv = lookup(kv, string("video"));
    string sfc = lookup(kv, string("cfg"));
    string sfcc = lookup(kv, string("camc"));
    cfg_.swd = lookup(kv, string("wdir"));
    //---- load cfg / camCfg
    if(!(cfg_.mcfg.load(sfc) 
        && cfg_.camc.load(sfcc) ) )
        return false;

    //---- load img or video
    Sp<Video> pv = nullptr;
    if(sfi!="")
        ok = run_pose_img(sfi);
    else if(sfv!="")
        ok = run_pose_video(sfv);
    else{
        log_e("video or img source not provided");
        return false;
    }
    return ok;
}
//-----------
bool CmdMarker::run_pose_img(CStr& sf)
{
    auto p = Img::create();
    if(!p->load(sf))
        return false;
    auto& im = *p;
    vector<Marker> ms;

    bool ok = pose_est(im, ms);
    //--- write output
    CStr& swd = cfg_.swd;
    if(swd=="") return ok;
    
    string sfw = swd +"/" + fn::nopath(sf);
    im.save(sfw);
    
}
//-----------
bool CmdMarker::run_pose_video(CStr& sf)
{
    //--- open video
    vector<Marker> ms;

    auto pv = Video::open(sf);
    if(pv==nullptr)
        return false;
     //------------
    // handle video
    //------------
    bool ok = true;
    Sp<Img> p;
    int i=1;
    while((p=pv->read()) && (p!=nullptr))
    {
        log_i("-- Video Frame:"+to_string(i++));
        ok &= pose_est(*p, ms);
    }
    return ok;    
}

//-----------
bool CmdMarker::pose_est(Img& im, vector<Marker>& ms)
{
    bool ok = true;
    auto& camc = cfg_.camc;
    auto& mcfg = cfg_.mcfg;
    ok = Marker::detect(im, mcfg, camc, ms);
    stringstream ss;
    ss << "Found markers:" << ms.size() << endl;
    for(auto& m : ms)
        ss << m.str() << endl;
    log_i(ss.str());

    //---- draw info on img
    draw(im, ms);

    return ok;
}

//--------------
void CmdMarker::draw(Img& im, const vector<Marker>& ms)const
{
    auto& camc = cfg_.camc;

    //--- draw info on img
    for (auto &m : ms)
    {
        Pose Tcm = m.pose;
        Px px = toPx(camc.proj(Tcm.t));
        px += Px({-150,50});
        stringstream s;
        s << "id=" << m.id << ", ";
        s << "t=" << str(Tcm.t);
        im.text(s.str(), px, {50,200,255});
        //--- draw axis
        im.axis(camc,Tcm, m.w ,2);
    }       
}
