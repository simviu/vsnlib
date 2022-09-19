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
    struct LCfg{
        string sf_omni_st = "omni_stereocalib_data.xml";
    }; LCfg lc_;
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
    log_i("Calib omni stereo...");

    string sfom = lc_.sf_omni_st;
    log_i("Loading :"+ sfom + "...");
    cv::FileStorage fs(sfom, cv::FileStorage::READ);

    std::vector<cv::Mat> objectPoints, imagePoints;
    cv::Size imgSize;
    fs["objectPoints"] >> objectPoints;
    fs["imagePoints"] >> imagePoints;
    fs["imageSize"] >> imgSize;
    //----
    log_e("Under construction...");
    return false;
}
