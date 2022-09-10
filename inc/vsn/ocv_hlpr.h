/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#pragma once

#include "vsn/eigen_hlpr.h"

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



#include "vsn/vsnLib.h"

namespace ocv{
    using namespace cv;
    using namespace std;
 //  sing namespace ut;
    using namespace vsn;

    extern string str(const cv::Point3d& d, int decim);

    //---- dbg show OpenCV img
    extern bool dbg_showImgWind();
    
    //---------
    // conversion
    //---------
    inline Scalar toCv(const Color& c)
    { return Scalar(c.b, c.g, c.r); }
    inline Scalar toCv(const BGR& c)
    { return Scalar(c.b, c.g, c.r); }
    inline Scalar toCv(const HSV& c)
    { return Scalar(c.h, c.s, c.v); }
    inline Point2d toCv(const vec2& v)
    { return Point2d(v.x(), v.y()); }
    inline Point3d toCv(const vec3& v)
    { return Point3d(v.x(), v.y(), v.z()); }
    inline vec2 toVec(const Point2d& c)
    { vec2 v; v<<c.x, c.y; return v;}
    inline vec2 toVec(const Point2f& c)
    { vec2 v; v<<c.x, c.y; return v;}
    inline vec4 toVec4(const cv::Mat& v)
    { vec4 e; 
      e << v.at<double>(0,0), v.at<double>(1,0), 
           v.at<double>(2,0), v.at<double>(3,0);
      return e;
    }
    
    
    inline cv::Rect toCv(const ut::Rect& r)
    {   auto& c = r.cntr;  auto& sz = r.sz;
        float w = sz.w; float h = sz.h;
      return cv::Rect(Point(c.x -w*0.5, c.y-h*0.5),  Size(w, h));
    }
    inline ut::Rect toUt(const cv::Rect& r)
    { return {{(int)(r.x + r.width *0.5), 
               (int)(r.y + r.height*0.5)},
               {r.width, r.height}}; }


}