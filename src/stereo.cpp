#include "vsn/vsnLib.h"
#include "vsn/ocv_hlpr.h"
using namespace vsn;

bool StereoVO::onImg(const Img& im1,  
                     const Img& im2)
{

    auto& imc1 = reinterpret_cast<const ocv::ImgCv*>(&im1)->im_;
    auto& imc2 = reinterpret_cast<const ocv::ImgCv*>(&im2)->im_;

    bool ok = true;
    FeatureMatch fm;
    fm.cfg_.bShow = cfg_.bShow;
    ok &= fm.onImg(im1, im2);

    //---------------
    // Setting Ref : 
    //   https://jayrambhia.com/blog/disparity-mpas
    //
    /*
        sgbm.SADWindowSize = 5;
        sgbm.numberOfDisparities = 192;
        sgbm.preFilterCap = 4;
        sgbm.minDisparity = -64;
        sgbm.uniquenessRatio = 1;
        sgbm.speckleWindowSize = 150;
        sgbm.speckleRange = 2;
        sgbm.disp12MaxDiff = 10;
        sgbm.fullDP = false;
        sgbm.P1 = 600;
        sgbm.P2 = 2400;
    */
    
    auto p_sgbm = cv::StereoSGBM::create(
        -64, //  int minDisparity = 0, 
        16, // int numDisparities = 16, 
        3, // int blockSize = 3,
        600,  // int P1 = 0, 
        2400, // int P2 = 0, 
        10, // int disp12MaxDiff = 0,
        4, // int preFilterCap = 0, 
        1,// int uniquenessRatio = 0,
        150, // int speckleWindowSize = 0, 
        2 // int speckleRange = 0,
        // int mode = StereoSGBM::MODE_SGBM
    );
    auto& sgbm = *p_sgbm;

    


    //---------------
    cv::Mat im_disp, im_disp2;
    sgbm.compute(imc1, imc2, im_disp);

    //---- display
    cv::normalize(im_disp, im_disp2, 0, 255, cv::NORM_MINMAX, CV_8U);

    if(cfg_.bShow)
    {
        string sName = "depth";
        cv::namedWindow(sName, cv::WINDOW_KEEPRATIO);
        imshow(sName, im_disp2);
        cv::waitKey(10);
    }
    return ok;
}