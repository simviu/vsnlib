/*
   Author: Sherman Chen
   Create Time: 2022-09-07
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/vsnTool.h"
#include "vsn/vsnLibCv.h"

using namespace app;

//----
namespace{
}

//----
CmdCalib::CmdCalib():
    Cmd("Calib commands")
{
    //---- 'omni_stereo'
    {
        string sH = "Omnidir Stereo Calibration \n";
        sH += "   Usage: omni_stereo \n";
        add("omni_stereo", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_omni_stereo(args); }));
    }
    //---- 'stereo'
    {
        string sH = "Stereo Calibration \n";
        sH += "   Usage: stereo dir=<DIR>\n";
        add("stereo", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_stereo(args); }));
    }
    //---- 'Info'
    {
        string sH = "Camera info \n";
        sH += "   Para: file=<CM_YAML_FILE>\n";
        add("info", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_cam_info(args); }));
    }
}
//-------
bool CmdCalib::run_cam_info(CStrs& args)
{
    StrTbl kv; parseKV(args, kv);
    string sf = lookup(kv, string("file"));
    CamCfg camc;
    if(!camc.load(sf)) return false;
    CamCfg::Lense l;
    camc.toLense(l);
    log_i(l.str());
    return true;
}

