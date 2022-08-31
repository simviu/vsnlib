#include "vsn/vsnTest.h"

#include <stdio.h>
#include <iostream>
#include <opencv2/features2d.hpp>



#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <memory>

#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace vsn;
using namespace ut;
using namespace test;
using namespace cv;

namespace {
    //-----
    struct LCfg{
        string sf_img = "blob_td/1.jpg";
    }; LCfg lc_;
    ///-------
    void test_blob_sample()
    {
            
        using namespace cv;
        // Read image
        Mat im = imread( lc_.sf_img, cv::IMREAD_GRAYSCALE );

        // Set up the detector with default parameters.
        auto p_det = SimpleBlobDetector::create();
        auto& detector = *p_det;

        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector.detect( im, keypoints);

        // Draw detected blobs as red circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        Mat im_with_keypoints;
//      drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255) );

        // Show blobs
        imshow("keypoints", im_with_keypoints );
        waitKey(0);
    
    }
}

//-----
bool TestBlob::run()
{
    test_blob_sample();
    return true;
}