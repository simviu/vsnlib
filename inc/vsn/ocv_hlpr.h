/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
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
    using namespace ut;
    using namespace vsn;

    extern string str(const cv::Point3d& d, int decim);

    //---- dbg show OpenCV img
    extern bool dbg_showImgWind();
    
    //---------
    // conversion
    //---------
    inline Scalar toCv(const Color& c)
    { return Scalar(c.b, c.g, c.r); }
    inline Point2f toCv(const vec2& v)
    { return Point2f(v.x(), v.y()); }
    inline vec2 toVec(const Point2f& c)
    { vec2 v; v<<c.x, c.y; return v;}
    //------------
    // ImgCv
    //------------
    // Implementation of Img
    struct ImgCv : public vsn::Img{
        ImgCv(){}
        ImgCv(cv::Mat im):im_(im){};
        ImgCv(const Img& im)
        {  
            im_ = *reinterpret_cast<const cv::Mat*>(im.data());
        }

        virtual bool load(ut::CStr& s) override;
        virtual bool save(ut::CStr& s) override;
        virtual bool val()const override
        { return (im_.rows>0) && (im_.cols>0);}
        virtual void show(CStr& sWind)override;

        virtual void text(CStr& s, 
            const Px& px={30,30},
            const Color& c={255,255,255})override;
       virtual void line(const Line2d& l,
                          const Color& c, 
                          double w=1.0)override;
        virtual void axis(const CamCfg& cc,
                const Pose& p, double l, double w)override;

        //-----
        virtual void* data()override
        { return reinterpret_cast<void*>(&(im_)); }
        virtual const void* data()const override
        { return reinterpret_cast<const void*>(&(im_)); }
        //---- dict selection ref OpenCV ArUco.
        // default is 5x5_250, TODO: change dict
     //   virtual void detect(vector<vsn::Marker>& markers)override;

        virtual void undistort(const CamCfg& cc)override;
        cv::Mat im_;
    protected:
    };

}