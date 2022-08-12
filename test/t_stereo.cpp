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
//using namespace cv;

namespace{
    const struct{
        string sf_cap = "testd/cur_video.m4v";
    }lcfg_;
}
//--------------------------
bool TestStereo::run()
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

        // Display the resulting frame
        cv::imshow( "Frame", frame );

        // Press  ESC on keyboard to exit
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





