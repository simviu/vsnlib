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
    //---- 'omni_st'
    {
        string sH = "Omnidir Stereo Calibration \n";
        sH += "   Usage: omni_st \n";
        add("omni_st", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_omni_stereo(args); }));
    }

}

//------
bool CmdCalib::run_omni_stereo(CStrs& args)
{
    return true;
}
