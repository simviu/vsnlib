/*
   Author: Sherman Chen
   Create Time: 2022-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
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
        bool load(CStr& sf);
        vec2 proj(const vec3& p)const;
        Line2d proj(const Line& l)const;
        
        //---- camera distortion para
        struct Dist{
            Dist(){}
            Dist(const vec5& v): k1(v(0)),k2(v(1)),
            p1(v(2)),p2(v(3)),k3(v(4)){}
            double k1=0;
            double k2=0;
            double k3=0;
            double p1=0;
            double p2=0;
            vec5 V()const
            { vec5 v; v << k1,k2,p1,p2,k3; return v;}
            string str()const;
        };


        //---- Camera lens para
        struct Lense{
            double cx=0, cy=0, fx=0, fy=0;
            double fovh = 0;
            double fovv = 0;
            double fov = 0;

            string str()const;
        };
        //---- functions
        void undis(const vec2s& vds, vec2s& vs)const;
        bool toLense(Lense& l)const;
    
        //----------- data -----------
        //--- camera intrinsic 3x3
        mat3 K;
        //--- camera distortion
        Dist D; 
        //---- camera dimention
        Sz sz; 

    };
    //---- streamming
    inline ostream& operator <<(ostream& s, const CamCfg::Dist& d)
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

        virtual bool load(CStr& s, int cvFlag=1)=0;
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
        virtual void toGray()=0;
        //----
        static Sp<Img> create();
        //---- internal storage data (Mat)
        virtual void* data()=0;
        virtual const void* data()const=0;
        //--- Suggest undistortion at very beginning
        virtual void undistort(const CamCfg& cc)=0;
        virtual Sp<Img> copy()const =0;
        //---- img operations
        virtual void rot(double dgr)=0;
        //---- 
        static Sp<Img> loadFile(const string& sf, int cvFlags=1);
    protected:
    };
    //-------------
    // video
    //-------------
    class Video{
    public:
        Video(){}
        struct Cfg{
            Sz sz;
            float fps=1;
        };

        static Sp<Video> open(CStr& s);
        static Sp<Video> create(CStr& sf, const Cfg& cfg);
        virtual Sp<Img> read()=0;
        virtual bool write(const Img& im)=0;
        Cfg cfg_;
    };


    //-----------
    // marker (ArUco)
    //-----------
    struct Marker{
        int id = 0;
        int dict_id=0; // ArUco dictionary id
        vec2 ps[4]; // 4 corner pos on image 
        //----
        double w=0.0001; // marker width
        Pose pose; // estimated pose, relative to camera
        //---- user define cfg before pose estimate
        //---------------
        // Pose Estimator
        //---------------
        class PoseEstimator{
        public:
            //---- Marker cfg
            struct MCfg{
                string sDict_="aruco_dict_id0";
                int dict_id_=0;
                struct Grp{
                    set<int> ids;
                    double w=1;
                };
                vector<Grp> grps_;
                //--- load json def file
                bool load(CStr& sf);
                string str()const;
            };
            //--- cfg
            struct Cfg{
                //--- cam cfg
                CamCfg camc;
                //--- Marker cfg
                MCfg mcfg;
            };
            Cfg cfg_;
            //---- result
            struct{ vector<Marker> ms; }result_;
            //---- detect
            bool onImg(const Img& im);
        };
        // (TODO:deprecated) Call back function that retrieve
        // marker width for pose estimation.
        // Defulat null, w=1.0 
        //using FWidthCb=std::function<double(int id)>;
        //---- default dict_id=6, cv::aruco::DICT_5X5_250
        static bool detect(const Img& im, vector<Marker>& ms,
                            int dict_id = 6); 
        
        //---- pose estimate, wid : marker width
        bool pose_est(const CamCfg& camc, double wid);
        
        //--- fit plane
        static bool fit_plane(
            const vector<Marker>& ms, Pose& p);
        //--- print
        string str()const;
    };

    //----------
    // FeatureMatch
    //----------
    class FeatureMatch{
    public:
        static Sp<FeatureMatch> create();
        struct Cfg{
            bool bShow = false;
            double distTH = 30;
            int N=100;
        };
        Cfg cfg_;
        //----
        struct Match{
            vec2 p1, p2;
        };
        //----
        struct Data{ 
            vector<Match> ms;
        };
        Data data_;
        //----
        virtual bool onImg(const Img& im1,
                           const Img& im2)=0;
    };
    //------------
    // StereoVO
    //------------
    //Stereo video odometry
    class StereoVO{
    public:
        StereoVO(){ reset(); }
        static Sp<StereoVO> create();
        //----
        struct Cfg{
            CamCfg camc;
            bool bShow = false;
            double baseline = 0.50;
            struct Odom{
                // 1:triangulation , 2:depth
                int mpnt_sel=1;
                // z threshold
                double z_TH = 50;
            }; Odom odom;
            struct Feature{
                int Nf = 100;
            }; Feature feature;
            //----
            bool load(const string& sf);
            string str()const;
        };
        Cfg cfg_;
        //----
        struct Data{
            // local points by stereo matching 
            //   and triangulations.
            //---
            struct Odom{
                
                mat3 R;
                vec3 t;
                vec3 e; // euler angle
                void reset()
                { R = mat3::Identity(); t << 0,0,0; e << 0,0,0; }
            }; Odom odom;

            void reset(){ odom.reset(); }           
            //---- depth disparity map
            Sp<Img> p_imd_ = nullptr;

        };
        Data data_;
        //----
        virtual bool onImg(const Img& im1, 
                           const Img& im2)=0;

        virtual bool genDepth(const Img& im1,  
                              const Img& im2)=0;
        virtual void reset();
    protected:

    };

}

