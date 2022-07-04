/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
 */

#pragma once

#include <stdio.h>
#include <iostream>
#include "vsn/cutil.h"
#include "vsn/eigen_hlpr.h"


namespace vsn{
    using namespace std;
    using namespace ut;
    using namespace egn;

    //------------
    // primitive
    //------------
    struct Line2d{
         Line2d(){ p1.Zero(); p2.Zero(); }
         vec2 p1; vec2 p2; 
         string str()const ;
    };
    struct Line{
         Line(){ p1.Zero(); p2.Zero(); }
         vec3 p1; vec3 p2; 
         string str()const ;
    };
    //-----
    struct Pose{ 
        quat q; 
        vec3 t; 
        string str()const;
        Pose inv()const;
        vector<Line> axis(double l=1.0)const;
        Pose operator *(const Pose& p)const;
    };

    //---------
    // utils
    //---------
    //---- open CV wait key
    extern int cv_waitkey(int MS);
    extern bool cv_waitESC(int MS);


    //---------
    // cam
    //---------
    struct CamCfg{
        using Ptr = shared_ptr<CamCfg>;
        bool load(CStr& sf)
        { return data_.load(sf); }
        vec2 proj(const vec3& p)const;
        Line2d proj(const Line& l)const;
        // camera distortion para
        struct TDist{
            TDist(){}
            TDist(const vec5& v): k1(v(0)),k2(v(1)),
            p1(v(2)),p2(v(3)),k3(v(4)){}
            double k1=0;
            double k2=0;
            double k3=0;
            double p1=0;
            double p2=0;
            vec5 V()const
            { vec5 v; v << k1,k2,p1,p2,k3; }
            
        };
        //---
        struct Data{
            mat3 K;
            //vec5 D;
            TDist D; // distortion

            int W=0;
            int H=0;
            bool load(CStr& sf);

        };
        Data data_;
    };
    //---- streamming
    inline ostream& operator <<(ostream& s, const CamCfg::TDist& d)
    { s << d.V(); return s;}
    //------------
    // Camera
    //------------
    class Cam{
        Sp<CamCfg> pCamc=nullptr;
        Pose pose_;
    };

    //------------
    // Img
    //------------
    struct Img{
        Img(){}
        typedef shared_ptr<Img> Ptr;
        typedef shared_ptr<const Img> CPtr;

        virtual bool load(CStr& s)=0;
        virtual bool save(CStr& s)=0;
        virtual bool val()const =0;
        operator bool(){ return val(); }

        virtual void show(CStr& sWind)=0;
        //---- draw functions
        virtual void text(CStr& s, 
            const Px& px={30,30},
            const Color& c={255,255,255})=0;
        virtual void line(const Line2d& l,
                          const Color& c={255,255,255}, 
                          double w=1.0)=0;
        virtual void axis(const CamCfg& cc,
            const Pose& p, double l=1.0, double w=1.0)=0;
        //----
        static Sp<Img> create();
        //---- internal storage data (Mat)
        virtual void* data()=0;
        virtual const void* data()const=0;
        //--- Suggest undistortion at very beginning
        virtual void undistort(const CamCfg& cc)=0;
    protected:
    };
   


    //-----------
    // marker (ArUco)
    //-----------
    struct Marker{
        int id = 0;
        vec2 ps[4]; // 4 corner pos on image 
   //     double w=1.0; // marker width
        Pose pose; // estimated pose, relative to camera

        // Call back function that retrieve
        // marker width for pose estimation.
        // Defulat null, w=1.0 
        using FWidthCb=std::function<double(int id)>;
        static bool detect(const Img& im, vector<Marker>& ms);
        //---- pose estimate, w : marker width
        bool pose_est(const CamCfg& camc, double w);

        //--- fit plane
        static bool fit_plane(
            const vector<Marker>& ms, Pose& p);
        //--- print
        string str()const;
    };
   
   

}

