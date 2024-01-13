/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */
#pragma once

#include "vsnLib.h"
#include "ocv_hlpr.h"


namespace vsn
{


    //------------
    // cfg
    //------------
    /*
    class UnDistMapImpl : public CamCfg::UnDistMap{
    public:
        virtual Sp<Img> remap(const Img& img)override;
        cv::Mat map1, map2;
    };
    */
    //------------
    // ImgCv
    //------------
    // Implementation of Img
    struct ImgCv : public vsn::Img{
        ImgCv(){}
        ImgCv(cv::Mat& im):im_(im){};
        ImgCv(const Img& im)
        {  
            im_ = *reinterpret_cast<const cv::Mat*>(im.data());
        }

        virtual int type()override
        { return im_.type(); }

        virtual Sz size()const override
        {  return {im_.cols,im_.rows}; }

        virtual bool load(ut::CStr& s, int cvFlag) override;
        virtual bool save(ut::CStr& s)const override;
        virtual bool val()const override
        { return (im_.rows>0) && (im_.cols>0);}
        virtual void show(CStr& sWind)const override;
        //---
        virtual void set(const Px& px, const Color& c) override;
        virtual bool get(const Px& px, Color& c)const override;
        virtual void set(const Px& px, const HSV& c) override;
        virtual bool get(const Px& px, HSV& c)const override;

        virtual void draw(CStr& s, 
                          const Px& px={30,30},
                          const Color& c={255,255,255},
                          float font_scl=1.0)override;
        virtual void draw(const vec2s& vs, const Color& c, 
                          float w=1.0)override;  
        virtual void draw(const vector<Line2d>& lns,
                          const Color& c={255,255,255}, 
                          double w=1.0)override;
        virtual void draw(const ut::Rect& r, const Color& c, 
                          float w=1.0)override;
        virtual void draw(const vector<Circle>& cs, const Color& c, 
                          float w=1.0)override;
        virtual void toGray()override;
        virtual void toHsv()override;
        virtual void blur(int w)override;
        virtual void filter(const HSV& c0,
                            const HSV& c1) override;
        virtual void scale(const Sz& sz, int method)override;
        //-----
        virtual void* data()override
        { return reinterpret_cast<void*>(&(im_)); }
        virtual const void* data()const override
        { return reinterpret_cast<const void*>(&(im_)); }
        virtual Sp<Img> copy()const override
        {  auto p = mkSp<ImgCv>(); 
           im_.copyTo(p->im_); return p;  }
        virtual Sp<Img> crop(const ut::Rect& r)const override;
        
        virtual void rot(double dgr)override;

        //---- dict selection ref OpenCV ArUco.
        // default is 5x5_250, TODO: change dict
     //   virtual void detect(vector<vsn::Marker>& markers)override;

        virtual void undistort(const CamCfg& cc)override;
        cv::Mat im_;
        cv::Mat raw(){ return im_; }
        cv::Mat raw()const{ return im_; }
        virtual vector<Line2d> det(const HoughLnCfg& c)const override;
        virtual vector<Circle> det(const HoughCirCfg& c)const override;

    protected:
    };
    //---- cast utils
    inline cv::Mat img2cv(const Img& im)
    { ImgCv imc(im); return imc.im_; }
    //------------
    // VideoCv
    //------------
    // Implementation of Video
    class VideoCv : public vsn::Video{
    public:
        VideoCv(){};
        VideoCv(int idx, Sz res=Sz(-1,-1)); // cam index
        VideoCv(CStr& s);
        virtual Sp<Img> read()override;
        
        bool isOpen() { return cap_.isOpened(); }
        bool createWr(CStr& sf);
        virtual bool write(const Img& im)override;
        virtual void close()override;

    protected:
        cv::VideoCapture cap_;
        Sp<cv::VideoWriter> p_vwr = nullptr;
        void init();

    };

    //----------
    // FeatureMatchCv
    //----------
    class FeatureMatchCv : public FeatureMatch{
    public:
        //---- Features
        struct Features{
            cv::Mat desc; // feature desripiton
            vector<cv::KeyPoint> pnts; // feature points
            void onImg(const ImgCv& im);
        };
        //---- MatchDt
        struct MatchDt{
            vector<cv::DMatch> dms;
            // lookup map keypnt index to matches index
            map<int, int> i1_mi;
            map<int, int> i2_mi;

        };
        
        //---- cv data
        struct Data{
            Features fs1, fs2;
            MatchDt md;
        };
        Data data_;
        //----
        virtual bool onImg(const Img& im1,
                           const Img& im2)override;
        bool match(const Features& fs1,
                   const Features& fs2,
                   MatchDt& md)const;
    };
    
    
}
