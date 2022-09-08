/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/ocv_hlpr.h"
#include "vsn/vsnLibCv.h"

#include <filesystem>
#include <opencv2/ximgproc/disparity_filter.hpp>
using namespace vsn;
using namespace ut;
using namespace test;
using namespace cv;

namespace
{
}
#define VIS_MULT 8
#define DOWNSCALE 1
#define MAX_DISPARITY 128
#define WIN_SIZE_SGBM 10
#define SMOOTHING_FACTOR 1
#define MAX_X_GRAD 63

#define LAMBDA 8000
#define SIGMA 2

//----
namespace{
    struct LCfg{
        string sf_L = "pair/1L.png";
        string sf_R = "pair/1R.png";
        string sf_camc    = "cfg/cam.yml";
        string sf_stereoc = "cfg/stereo.json";
    }; LCfg lc_;

}

//----------
// ref:
//   https://answers.opencv.org/question/223280/calculating-depth-with-ptrstereosgbm-black-bar-on-image/
bool test_LR(const Img& imL, const Img& imR)
{
    auto p_vo = StereoVO::create();
    auto& vo = *p_vo;
    if (!vo.cfg_.load(lc_.sf_stereoc))
        return false;
    CamCfg camc;
    if (!camc.load(lc_.sf_camc))
        return false;
    vo.cfg_.camc = camc;

    //-------
    // run
    //-------
    vo.onImg(imL, imR);

    //----
    vo.showLoop();    

    return true;
};
//----------
// ref:
//   https://answers.opencv.org/question/223280/calculating-depth-with-ptrstereosgbm-black-bar-on-image/
bool test_LR_ref(const Img& imL, const Img& imR)
{

    // input
    cv::Mat left, right;
    left  = ImgCv(imL).raw();
    right = ImgCv(imR).raw();

    bool no_downscale = true;
    int max_disp = MAX_DISPARITY;
    
    Mat left_for_matcher, right_for_matcher;
    Mat left_disp, right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(left.rows, left.cols, CV_8U);
    conf_map = Scalar(255);
    cv::Rect ROI;
    Ptr<ximgproc::DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;

    if (!no_downscale)
    {
        max_disp *= DOWNSCALE;
        if (max_disp % 16 != 0)
            max_disp += 16 - (max_disp % 16);
        resize(left, left_for_matcher, Size(), DOWNSCALE, DOWNSCALE);
        resize(right, right_for_matcher, Size(), DOWNSCALE, DOWNSCALE);
    }
    else
    {
        left_for_matcher = left.clone();
        right_for_matcher = right.clone();
    }

    Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, max_disp, WIN_SIZE_SGBM);
    left_matcher->setP1(24 * WIN_SIZE_SGBM*WIN_SIZE_SGBM*SMOOTHING_FACTOR);
    left_matcher->setP2(96 * WIN_SIZE_SGBM*WIN_SIZE_SGBM*SMOOTHING_FACTOR);
    left_matcher->setPreFilterCap(MAX_X_GRAD);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
    wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

    matching_time = (double)getTickCount();
    left_matcher->compute(left_for_matcher, right_for_matcher, left_disp);
    right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
    matching_time = ((double)getTickCount() - matching_time) / getTickFrequency();

    wls_filter->setLambda(LAMBDA);
    wls_filter->setSigmaColor(SIGMA);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp, left, filtered_disp, right_disp);
    filtering_time = ((double)getTickCount() - filtering_time) / getTickFrequency();

    conf_map = wls_filter->getConfidenceMap();

    // Get the ROI that was used in the last filter call:
    if (!no_downscale)
    {
        // upscale raw disparity and ROI back for a proper comparison:
        resize(left_disp, left_disp, Size(), 1 / DOWNSCALE, 1 / DOWNSCALE);
        left_disp = left_disp / DOWNSCALE;
    }



    Mat raw_disp_vis;
    cv::ximgproc::getDisparityVis(left_disp, raw_disp_vis, VIS_MULT);

    Mat filtered_disp_vis;
    cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, VIS_MULT);

   // return filtered_disp_vis;
    imshow("confmap", conf_map);
    imshow("left", left);
    imshow("right", right);
    imshow("raw disparity vis", raw_disp_vis);
    imshow("filtered disparity vis", filtered_disp_vis);
    vsn::show_loop();
    return true;
}

//------------------
// test_imgLR()
//------------------
bool TestStereo::test_imgLR()const
{
    auto pL = Img::loadFile(lc_.sf_L);
    auto pR = Img::loadFile(lc_.sf_R);
    if(pL==nullptr || pR==nullptr)
        return false;

 // return test_LR_ref(*pL, *pR);
    return test_LR(*pL, *pR);

    return true;
}
