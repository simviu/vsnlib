/*
   Author: Sherman Chen
   Create Time: 2022-08-31
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLibCv.h"

using namespace vsn;
using namespace ut;

//-----------------
// detect
//-----------------
bool Instance::detect(const Img& im)
{
    using namespace cv;
    auto p1 = im.copy();
    auto& im1 = *p1;
    auto& fc = cfg_.filter;
    im1.filter(fc.c0, fc.c1);

    ImgCv im2(im1);
    Mat imf = im2.im_;
    Mat imb, imt;

    // pre-process
    float bsz = cfg_.blurSz;
    blur(imf, imb, Size(bsz, bsz)); // apply blur to grayscaled image
    threshold(imb, imt, 50, 255, THRESH_BINARY); // apply binary thresholding

    // find contours
    vector< vector<Point> > contrs; // list of contour points
    vector<Vec4i> hier;
    findContours(imt, contrs, hier, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    // create hull array for convex hull points
    int N = contrs.size();
    vector< vector<Point> > hull(N);
    vector<cv::Rect> bboxes;
    
    for(int i = 0; i < contrs.size(); i++)
    {
        convexHull(Mat(contrs[i]), hull[i], false);
        cv::Rect b = boundingRect(hull[i]);  
        if(b.area() > cfg_.areaTH)
            bboxes.push_back(b);  
    }

    //---- show result
    if(!cfg_.enShow)
        return true;

    // show process image
    im.show("input");
    im1.show("filter");
    imshow("blur", imb);
    imshow("threshold", imt);

    // result
    // create a blank image (black image)
    //Mat imo = Mat::zeros(imt.size(), CV_8UC3); 
    auto p_imo = im.copy();
    ImgCv imoc(*p_imo);
    cv::Mat imo = imoc.im_;
    Scalar cc = Scalar(0, 255, 0); // green - color for contours
    Scalar ch = Scalar(255, 0, 0); // blue - color for convex hull
    Scalar cb = Scalar(0,255,0); // bbox
    for(int i = 0; i < N; i++) {
        // draw ith contour
        drawContours(imo, contrs, i, cc, 1, 8, vector<Vec4i>(), 0, Point());
        // draw ith convex hull
        drawContours(imo, hull, i, ch, 3, 8, vector<Vec4i>(), 0, Point());
        // Bounding box
        //rectangle(imo,  bbox[i], cb, 2);
    }
    for(auto& b : bboxes)
        rectangle(imo,  b, cb, 2);


    //----
    imshow("result", imo);

    //---- hold 
    while(!vsn::cv_waitESC(10));
    
    return true;
}

