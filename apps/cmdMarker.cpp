#include "vsn/vsnTool.h"

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
bool CmdMarker::run_pose(CStrs& args)const
{
    bool ok = true;
    StrTbl kv;
    parseKV(args, kv);
    string sf = lookup(kv, string("img"));
    string sfc = lookup(kv, string("cfg"));
    string sfcc = lookup(kv, string("camc"));
    string swd = lookup(kv, string("wdir"));
    //---- load cfg / camCfg
    Marker::Cfg cfg;
    CamCfg camc;
    if(!(cfg.load(sfc) 
        && camc.load(sfcc) ) )
        return false;
    //---- load img and det
    auto p = Img::create();

    if(!p->load(sf))
        return false;
    vector<Marker> ms;
    ok = Marker::detect(*p, cfg, camc, ms);
    stringstream ss;
    ss << "Found markers:" << ms.size() << endl;
    for(auto& m : ms)
        ss << m.str() << endl;
    //----
    log_i(ss.str());
    //--- write output
    if(swd!="")
    {
        //--- draw info on img
        for (auto &m : ms)
        {
            Pose Tcm = m.pose;
            Px px = toPx(camc.proj(Tcm.t));
            px += Px({-150,50});
            stringstream s;
            s << "id=" << m.id << ", ";
            s << "t=" << str(Tcm.t);
            p->text(s.str(), px, {50,200,255});
        }       
        // ----write img
        
        string sfw = swd +"/" + fn::nopath(sf);
        p->save(sfw);
    }
    return ok;
}

