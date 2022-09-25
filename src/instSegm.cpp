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
    struct LCfg{
        // very small specle filter
        float box_area_TH = 30*30;
    }; LCfg lc_;
    //----
    void conv_pnts(const vector<cv::Point>& ps, vec2s& vs)
    { for(auto& p : ps)vs.push_back({p.x, p.y}); }
    void conv_pnts(const vec2s& vs, vector<cv::Point>& ps)
    { for(auto& v : vs)ps.push_back(cv::Point(v.x(), v.y())); }
    //------
    //------------
    // for red block test case
    Sp<Img> hsv_hue_shift90(const Img& imi)
    {
        auto p = imi.copy();
        auto& im = *p;
        Sz sz = im.size();
        for(int y=0;y<sz.h;y++)
            for(int x=0;x<sz.w;x++)
            {
                Px px(x,y);
                HSV d; im.get(px, d);
                unsigned int h = d.h;
                // dbg
                if(h< 3 || h > 175)
                { int k=0; }
                //
                h += 90;
                if(h>180) h -= 180;
               // h = 90; // dbg
                d.h = h;
                im.set(px, d);
            }
        //--- dbg
        if(0)
        {
            cv::Mat imc = ImgCv(im).raw();
            cv::Mat imd = imc;
            cv::cvtColor(imc, imd,cv::COLOR_HSV2BGR);
            imshow("Hue shift 90 RGB", imd);
            imwrite("hue90.jpg", imd);
        
        }
        return p;

    }

}

//-----------------
// detect
//-----------------
bool InstSegm::onImg(const Img& im)
{
    data_ = Data(); // clear data
    using namespace cv;
    Sz sz = im.size();
    
    auto& fc = cfg_.filter;
    auto p_imi = im.copy();
    p_imi->toHsv();
    auto p_imc = p_imi;
    if(cfg_.enHueShift90)
        p_imc = hsv_hue_shift90(*p_imi);
    
    p_imc->filter(fc.c0, fc.c1);
    cv::Mat imf = ImgCv(*p_imc).raw();    

    // pre-process
    cv::Mat imb = imf;
    float bsz = cfg_.blurSz;
    if(bsz>0)
    {
        blur(imf, imb, Size(bsz, bsz)); // apply blur to grayscaled image
        data_.p_imb = mkSp<ImgCv>(imb);
    }
    cv::Mat imt;
    threshold(imb, imt, 150, 255, THRESH_BINARY); // apply binary thresholding

    // find contours
    vector< vector<Point> > contrs; // list of contour points
    vector<Vec4i> hier;
    findContours(imt, contrs, hier, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    // create hull array for convex hull points
    int N = contrs.size();
    vector< vector<Point> > hull(N);
    vector<double> areas;
    //vector<cv::Rect> bboxes;
    
    for(int i = 0; i < N; i++)
    {
        convexHull(Mat(contrs[i]), hull[i], false);
        cv::Rect b = boundingRect(hull[i]);  
        //--- min speckle filter
        double ab = b.width*b.height;
        if(ab < lc_.box_area_TH)
            ab = -1;
        //---- filter contour ara
        double a=-1;
        if(ab > 0)
            a = contourArea(hull[i]);
        areas.push_back(a);
        if(a < cfg_.areaTH)
            continue;

        //---- filter result
        Inst in;
        in.box = ocv::toUt(b);  
        conv_pnts(hull[i], in.hull);
        data_.ins.push_back(in);
    }

    //---- show result
    if(!cfg_.en_imo)
        return true;

    // result
    // create a blank image (black image)
    //Mat imo = Mat::zeros(imt.size(), CV_8UC3); 
    auto p_imo = data_.p_imo;
    if(p_imo==nullptr)
    {
        p_imo = im.copy();
        data_.p_imo = p_imo;
    }
    ImgCv imoc(*p_imo);
    cv::Mat imo = imoc.im_;// bbox

    Scalar cc = Scalar(0, 255, 0); // green - contours
    Scalar ch = Scalar(255, 0, 0); // blue - convex hull
    Scalar ci = Scalar(0,255,0); // instance
    Color cp{255,0,0}; // Convex points
    //--- draw origin contours
    for(int i = 0; i < N; i++) {
        double a = areas[i];
        if(a<0) continue;

        // draw ith contour
        drawContours(imo, contrs, i, cc, 1, 8, vector<Vec4i>(), 0, Point());
        // draw ith convex hull
        drawContours(imo, hull, i, ch, 2, 8, vector<Vec4i>(), 0, Point());
        drawContours(imo, hull, i, ch, 2, 8, vector<Vec4i>(), 0, Point());
        // Bounding box
        //rectangle(imo,  bbox[i], cb, 2);
        //--- info of contour
        stringstream s;
        s << "a=" << a;
        Px px = ocv::toPx(hull[i][0])+Px(0, -10);
        p_imo->draw(s.str(), px, cp);

    }
    
    //---- draw instance
    cv::Mat im_cntr(sz.h, sz.w, CV_8UC1, {0,0,0});
    for(auto& in : data_.ins)
    {
        vector<Point> h; 
        conv_pnts(in.hull, h);
        vector<vector<Point>> hs{h};
        drawContours(imo, hs, -1, ci,  4, 8, vector<Vec4i>(), 0, Point());
        drawContours(im_cntr, hs, -1, {255,255,255}, 2, 8, vector<Vec4i>(), 0, Point());
        auto b = ocv::toCv(in.box);
        cv::rectangle(imo, b, ci, 2);
        //--- convex pnts
        p_imo->draw(in.hull, cp, 4);
    }
    data_.p_imc = mkSp<ImgCv>(im_cntr);
    data_.p_imt = mkSp<ImgCv>(imt);
    //----
 

    // show process image
    if(cfg_.enShow)
    {
    //    im.show("input");
        //p_imc_shft->show("HSV shift 90");
    //    imshow("filter", imf);
    //    imshow("blur", imb);
        imshow("threshold", imt);

        cv::imshow("InstSegm result", imo);
        cv::imshow("Inst Contours", im_cntr);
    }
    return true;
}

