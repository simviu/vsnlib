#include "vsn/vsnLibCv.h"
#include <opencv2/sfm/triangulation.hpp>

/*
*/


using namespace vsn;

//---- Factory
Sp<StereoVO> StereoVO::create()
{
    return mkSp<StereoVOcv>();
}
//-----------
bool StereoVOcv::onImg(const Img& im1,  
                       const Img& im2)
{
    auto& camc = cfg_.camc;

    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);

    bool ok = true;
    //---- do feature matching of L/R
    FeatureMatch fm;
    fm.cfg_.bShow = cfg_.bShow;
    fm.cfg_.N = 20;
    ok &= fm.onImg(im1, im2);

    //---- trangulate feature points.
    // ( inner arry for each image)
    vector<cv::Point2d> Qs1, Qs2;
    auto& ms = fm.result_.ms;
    int N = ms.size();
    for(auto& m : ms)
    {
        Qs1.push_back(ocv::toCv(m.p1));
        Qs2.push_back(ocv::toCv(m.p2));
    }
    //---- projection matrix P = K*T
    // We have 2 cameras.
    double b = cfg_.baseline;
    cv::Mat T1 = (cv::Mat_<double>(3,4) << 
            1, 0, 0,  -b*0.5,
            0, 1, 0,  0,
            0, 0, 1,  0);
    cv::Mat T2 = (cv::Mat_<double>(3,4) << 
            1, 0, 0,  b*0.5,
            0, 1, 0,  0,
            0, 0, 1,  0);

    cv::Mat K; cv::eigen2cv(camc.K, K); 
    cv::Mat Ps;
    cv::triangulatePoints(K*T1, K*T2, Qs1, Qs2, Ps);
    log_d("Triangulate pnts: "+to_string(N));

    //---- De-homoge
    stringstream s;
    for(int i=0;i<N;i++)
    {
        vec4 h = ocv::toVec4(Ps.col(i));
        vec3 v; 
        if(!egn::normalize(h, v))continue;
        s << v << ";  " << endl;
    }
    log_d(s.str());

    return ok;
}
//-----------
bool StereoVOcv::genDepth(const Img& im1,  
                          const Img& im2)
{
    ocv::ImgCv imc1(im1);
    ocv::ImgCv imc2(im2);

    bool ok = true;
   
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
    /* setting (1)
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
    */
    // Setting (2)
    auto p_sgbm =  cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); // tested parameters
    auto& sgbm = *p_sgbm;

    //---------------
    cv::Mat im_disp, im_disp2;
    sgbm.compute(imc1.im_, imc2.im_, im_disp);

    //---- display
    cv::normalize(im_disp, im_disp2, 0, 255, cv::NORM_MINMAX, CV_8U);
    //im_disp2 = im_disp*10;
    data_.p_imd_ = mkSp<ocv::ImgCv>(im_disp);

    if(cfg_.bShow)
    {
        string sName = "depth";
        cv::namedWindow(sName, cv::WINDOW_KEEPRATIO);
        imshow(sName, im_disp2);
        cv::waitKey(10);
    }
    return ok;
}