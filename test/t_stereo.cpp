/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/ocv_hlpr.h"

using namespace vsn;
using namespace ut;
using namespace test;
using namespace cv;

namespace{
    const struct{
        string sf_camc = "vsntd/cam.yml";
        string sf_seqL = "vsntd/seq/image_0";
        string sf_seqR = "vsntd/seq/image_1";
        double baseline = 0.255;
    }lcfg_;
}
//--------------------------
bool TestStereo::run()
{
    bool ok = true;
    log_i("run TestStereo()...");   
    vector<string> sfLs, sfRs;
    cv::glob(lcfg_.sf_seqL, sfLs);
    cv::glob(lcfg_.sf_seqR, sfRs);
    CamCfg camc;
    if(!camc.load(lcfg_.sf_camc))
        return false;
    
    //---- main loop
    int N = sfLs.size();
    if(N>sfRs.size())
        N = sfRs.size();
    log_i("seq has total imgs:"+to_string(N));
    
    //-----
    for(int i=0;i<N;i++)
    {
        //---- load image
        int lf = cv::IMREAD_GRAYSCALE;
        auto p1 = Img::loadFile(sfLs[i], lf);
        auto p2 = Img::loadFile(sfRs[i], lf);
        if((p1==nullptr)||(p2==nullptr))
        {
            log_e("Failed load image '"+ 
                sfLs[i] + "' or '" + sfRs[i] +"'");
            return false;
        }
        auto& im1 = *p1;
        auto& im2 = *p2;
        //---- stereo VO test
        StereoVO vo;
        
        vo.cfg_.bShow = true;
        vo.cfg_.camc = camc;
        vo.onImg(im1, im2);
        //------ Press  ESC on keyboard to exit
        char c=(char)cv::waitKey(25);
        if(c==27)
        break;
    }
    
    // Closes all the frames
    cv::destroyAllWindows();
        
    return ok;

}





