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

namespace{
    void conv_pnts(const vector<cv::Point>& ps, vec2s& vs)
    { for(auto& p : ps)vs.push_back({p.x, p.y}); }
    void conv_pnts(const vec2s& vs, vector<cv::Point>& ps)
    { for(auto& v : vs)ps.push_back(cv::Point(v.x(), v.y())); }
}

//-----------------
// detect
//-----------------
bool InstSegm::onImg(const Img& im)
{
    using namespace cv;
    
    auto& fc = cfg_.filter;
    auto p_imc = im.copy();
    p_imc->toHsv();
    p_imc->filter(fc.c0, fc.c1);
    cv::Mat imf = ImgCv(*p_imc).raw();    

    // pre-process
    cv::Mat imb, imt;
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
    //vector<cv::Rect> bboxes;
    
    for(int i = 0; i < contrs.size(); i++)
    {
        convexHull(Mat(contrs[i]), hull[i], false);
        cv::Rect b = boundingRect(hull[i]);  
        if(b.area() < cfg_.areaTH)continue;
        // filter result
        Inst in;
        in.box = ocv::toUt(b);  
        conv_pnts(hull[i], in.hull);
        data_.ins.push_back(in);
    }

    //---- show result
    if(!cfg_.en_imo)
        return true;

    // show process image
    im.show("input");
    imshow("filter", imf);
    imshow("blur", imb);
    imshow("threshold", imt);
    //imshow("hsv", imh);

    // result
    // create a blank image (black image)
    //Mat imo = Mat::zeros(imt.size(), CV_8UC3); 
    auto p_imo = im.copy();
    data_.p_imo = p_imo;

    ImgCv imoc(*p_imo);
    cv::Mat imo = imoc.im_;// bbox

    Scalar cc = Scalar(0, 255, 0); // green - contours
    Scalar ch = Scalar(255, 0, 0); // blue - convex hull
    Scalar ci = Scalar(0,255,0); // instance

    //--- draw origin contours
    for(int i = 0; i < N; i++) {
        // draw ith contour
        drawContours(imo, contrs, i, cc, 1, 8, vector<Vec4i>(), 0, Point());
        // draw ith convex hull
        drawContours(imo, hull, i, ch, 2, 8, vector<Vec4i>(), 0, Point());
        // Bounding box
        //rectangle(imo,  bbox[i], cb, 2);
    }
    //---- draw instance
    for(auto& in : data_.ins)
    {
        vector<Point> h; conv_pnts(in.hull, h);
        vector<vector<Point>> hs{h};
        drawContours(imo, hs, -1, ci, 4, 8, vector<Vec4i>(), 0, Point());
        auto b = ocv::toCv(in.box);
        cv::rectangle(imo, b, ci, 2);
    }
    //----
    //imshow("result", imo);
    //---- hold 
    //while(!vsn::cv_waitESC(10));
    
    return true;
}

