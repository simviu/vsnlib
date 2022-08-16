/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/vsnLibCv.h"

using namespace vsn;
using namespace ut;
using namespace test;
using namespace cv;

namespace{
    const struct{
        string sf_cap = "testd/cur_video.m4v";
    }lcfg_;
}
//--------------------------
bool TestFeature::run()
{
    bool ok = true;
    log_i("run TestFeature()...");   
    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap(lcfg_.sf_cap); 
    
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
        
    while(1){

        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;
    
        // If the frame is empty, break immediately
        if (frame.empty())
            break;
        //---- split and crop
        //cv::Mat im1 = frame(rang(), range());
        int w = 1920;
        int h = 1080;
        int dh = 246;
        Mat im1 = frame({dh, h/2+dh-1},{0, w/2-1});
        Mat im2 = frame({dh, h/2+dh-1},{w/2, w-1});

        //--- dbg
        rectangle(frame, {0,dh}, {w/2, h/2 + dh},
                    Scalar(0, 0, 250),
                    1, LINE_8);
        //----
        // Display the resulting frame
        //cv::imshow( "Frame", frame );
       // cv::imshow( "Left", im1 );
       // cv::imshow( "Right", im2 );

        auto p_im1 = mkSp<ocv::ImgCv>(im1);
        auto p_im2 = mkSp<ocv::ImgCv>(im2);
        //---
        auto p_fm = vsn::FeatureMatch::create();
        auto& fm = *p_fm;
        fm.cfg_.distTH = 25;
        fm.cfg_.distTH = 20;
        fm.cfg_.bShow = true;
        fm.onImg(*p_im1, *p_im2);
        auto& ms = fm.data_.ms;
        log_d("features match:"+to_string(ms.size()));
        //------ Press  ESC on keyboard to exit
        char c=(char)cv::waitKey(25);
        if(c==27)
        break;
    }
    
    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();
        
    return ok;

}





