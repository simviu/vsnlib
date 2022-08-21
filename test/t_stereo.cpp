/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/ocv_hlpr.h"
#include <filesystem>

using namespace vsn;
using namespace ut;
using namespace test;
using namespace cv;

namespace
{
    const struct
    {
        string sf_camc = "stereo_td/cam.yml";
        string sf_stereoc = "stereo_td/stereo.json";
        string sf_seqL = "seq/image_0";
        string sf_seqR = "seq/image_1";
        string sf_Tw = "Tw.txt";
        double baseline = 0.255;
    } lcfg_;
    //--------------
    //-------- get index from Kitti img filename
    int getIdx(const string& sf)
    {
        string s= std::filesystem::path(sf).stem();
        for(int i=0;i<s.length()-1;i++)
        {
            if(s[i]!='0') break;
            s[i] = ' ';
        }
        
        int i = std::stoi(s);
        return i;
    }
    //---- Kitti format T 3x4, for evaluation.
    string kitti_line(const cv::Mat& Rw, 
                      const cv::Mat& tw, 
                      int idx)
    {
        stringstream s;
        s.precision(16);
        s << std::fixed;
        s << idx ; // current frame index

        cv::Mat_<double> Tw(3,4);
        for(int i=0;i<3;i++)
        Rw.col(i).copyTo(Tw.col(i));
        tw.copyTo(Tw.col(3));
        for(int i=0; i<Tw.rows; i++)
        for(int j=0; j<Tw.cols; j++)
            s << " " << Tw.at<double>(i, j);
        s << endl;
        string sr = s.str();
        return sr;
    }

}
//--------------------------
bool TestKittiStereo::run()
{
    bool ok = true;
    log_i("run TestStereo()...");
    vector<string> sfLs, sfRs;
    cv::glob(lcfg_.sf_seqL, sfLs);
    cv::glob(lcfg_.sf_seqR, sfRs);
    CamCfg camc;
    if (!camc.load(lcfg_.sf_camc))
        return false;
    //---- load stereo cfg
    //---- stereo VO test
    auto p_vo = StereoVO::create();
    auto &vo = *p_vo;
    if (!vo.cfg_.load(lcfg_.sf_stereoc))
        return false;

    vo.cfg_.bShow = true;
    vo.cfg_.camc = camc;

    //---- Open Kitti output
    ofstream oftw(lcfg_.sf_Tw);
    if(!oftw.is_open())
    {
        log_ef(lcfg_.sf_Tw);
        return false;
    }

    //---- main loop
    int N = sfLs.size();
    if (N > sfRs.size())
        N = sfRs.size();
    log_i("seq has total imgs:" + to_string(N));

    //-----
    for (int i = 0; i < N; i++)
    {
        //---- load image
        int lf = cv::IMREAD_GRAYSCALE;

        auto p1 = Img::loadFile(sfLs[i], lf);
        auto p2 = Img::loadFile(sfRs[i], lf);
        if ((p1 == nullptr) || (p2 == nullptr))
        {
            log_e("Failed load image '" +
                  sfLs[i] + "' or '" + sfRs[i] + "'");
            return false;
        }
        auto &im1 = *p1;
        auto &im2 = *p2;

        //--- process img
        vo.onImg(im1, im2);

        //--- log Kitti Tw to file
        auto& odom = vo.getData().odom;
        cv::Mat Rw, tw;
        cv::eigen2cv(odom.R, Rw);
        cv::eigen2cv(odom.t, tw);
        int idx = getIdx(sfLs[i]);
        oftw << kitti_line(Rw, tw, idx);

        //------ Press  ESC on keyboard to exit
        char c = (char)cv::waitKey(25);
        if (c == 27)
            break;
    }

    // Closes all the frames
    cv::destroyAllWindows();

    return ok;
}
