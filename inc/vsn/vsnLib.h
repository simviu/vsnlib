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
            double p1=0;
            double p2=0;
            double k3=0;
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

        typedef shared_ptr<Img> Ptr;
        typedef shared_ptr<const Img> CPtr;

        virtual bool load(CStr& s, int cvFlag=1)=0;
        virtual bool save(CStr& s)=0;
        virtual bool val()const =0;
        operator bool(){ return val(); }

        virtual void show(CStr& sWind)const=0;
        //---- draw functions, 
        //   TODO: replace with draw()
        virtual void text(CStr& s, 
            const Px& px={30,30},
            const Color& c={255,255,255})=0;
        
        virtual void line(const Line2d& l,
                          const Color& c={255,255,255}, 
                          double w=1.0)=0;
        virtual void axis(const CamCfg& cc,
            const Pose& p, double l=1.0, double w=1.0)=0;
        virtual void draw(const Rect& r, const Color& c, 
                          float w=1.0)=0;
        //----
        virtual void toGray()=0;
        virtual void toHsv()=0;
        virtual void filter(const Color& c0,
                            const Color& c1)=0;
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
        virtual void close()=0;
        Cfg cfg_;
    };

    //----------
    // Instance 
    //----------
    // Object instancing by color filter,
    //   Convex hull and mix of 
    //   OpenCV methods, without DNN.
    class Instance{
    public:
        struct Cfg{
            bool enShow = false;
            struct Filter{
                Color c0,c1;
            }; Filter filter;
            float blurSz = 3;
            float areaTH = 20*10;
        }; Cfg cfg_;
        struct Data{

        }; Data data_;
        bool detect(const Img& im);

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
    // Points
    //------------
    class Points{
    public:
        bool load(const string& sf);
        bool save(const string& sf);
        bool show()const;
    };

    //------------
    // StereoVO
    //------------
    //Stereo video odometry
    class StereoVO{
    public:

        static Sp<StereoVO> create();
        //----
        struct Cfg{
            CamCfg camc;
          //  bool bShow = false;
            double baseline = 0.50;
            struct Odom{
                // 1:triangulation , 2:depth
                int mode=1;
                // z threshold
                double z_TH = 50;
            }; Odom odom;
            struct Feature{
                int Nf = 100;
            }; Feature feature;
            struct Run{
                bool bShow=false;
                bool enDepth = false;
                bool enWr = false;
            }; Run run;
            struct PointCloud{
                double z_TH = 40;
            }; PointCloud pntCloud;

            //----
            bool load(const string& sf);
            string str()const;
        };
        Cfg cfg_;
        //---- Frm data
        struct Frm{
            // Triangulated feature points
            //  in global space.
            vec3s Pws; 
        };
        //----
        struct Data{
            int frmIdx = 0;
            //----
            struct Wr{
                ofstream ofs_xyz;
                ofstream ofs_Tw;
                bool open();
                void close();
            }; Wr wr;
            // local points by stereo matching 
            //   and triangulations.
            //---
            struct Odom{
                Odom(){ reset(); }
                mat3 Rw;
                vec3 tw;
                vec3 ew; // euler angle
                void reset()
                { Rw = mat3::Identity(); tw << 0,0,0; ew << 0,0,0; }
            }; Odom odom;
            //---- current Frm result
            Sp<Frm> p_frm = nullptr;
            //---- depth disparity map
            Sp<Img> p_imd_ = nullptr;

            // wr data
            bool wrData();
            void close(){ wr.close(); }
        };
        //----
        virtual bool onImg(const Img& im1, 
                           const Img& im2)=0;

        virtual bool genDepth(const Img& im1,  
                              const Img& im2)=0;

        auto& getData()const{ return data_; }
        void onFinish(){ data_.close(); }
        void setFrmIdx(int i){ data_.frmIdx=i; }
    protected:
        Data data_;
    };

}

