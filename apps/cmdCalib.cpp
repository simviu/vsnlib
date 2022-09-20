/*
   Author: Sherman Chen
   Create Time: 2022-09-07
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/vsnTool.h"
#include "vsn/vsnLibCv.h"


#include <opencv2/ccalib/multicalib.hpp>

using namespace app;

//----
namespace{
    struct LCfg{
        string sf_imL = "L.jpg";
        string sf_imR = "R.jpg";
    }; LCfg lc_;
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
        string sH = "Omnidir Stereo Calibration \n";
        sH += "   Usage: stereo \n";
        add("stereo", mkSp<Cmd>(sH,
        [&](CStrs& args)->bool{ return run_stereo(args); }));
    }
}

//------
bool CmdCalib::run_omni_stereo(CStrs& args)
{
    log_i("Calib omni stereo...");

    std::vector<cv::Mat> objectPoints, imagePoints1, imagePoints2;
    cv::Size imgSize1, imgSize2;
   
   
    //---- find pattern
    // TODO: find pattern and points
   //----
    cv::Mat K1, K2, xi1, xi2, D1, D2, idx;
    int flags = 0;
    cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
    std::vector<cv::Mat> rvecsL, tvecsL;
    cv::Mat rvec, tvec;
    double rms = cv::omnidir::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, imgSize1, imgSize2, K1, xi1, D1, K2, xi2, D2, rvec, tvec, rvecsL, tvecsL, flags, critia, idx);
   //-----
    stringstream s;
    vector<cv::Mat> pnts0{objectPoints[0], imagePoints1[0], imagePoints2[0]};
    s << "objPnts[0], imagePoints1[0], imagePoints2[0]:";
    for(auto& p : pnts0)
        s <<"(" << p.rows << "x" << p.cols << "), ";
    //---
    s << "---- calib result ----" << endl;
    s << "K1:" << endl << K1 << endl;
    s << "K2:" << endl << K2 << endl;
    s << "D1:" << endl << D1 << endl;
    s << "D2:" << endl << D2 << endl;
    s << "xi1:" << endl << xi1 << endl;
    s << "xi2:" << endl << xi2 << endl;
    s << "rvec:" << endl << rvec << endl;
    s << "tvec:" << endl << tvec << endl;
    s << "   rms=" << rms << endl;
    log_i(s.str());
    log_e("Under construction...");
    return false;
}


//------
bool CmdCalib::run_stereo(CStrs& args)
{
    log_i("Calib stereo...");
    return true;
}
