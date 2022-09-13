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
    //--------
    // types
    //--------
    //---- BGR can be used to access image BGR
    // elements directly.
    struct BGR {
        uint8_t b=0;
        uint8_t g=0;
        uint8_t r=0;  
        BGR(){}
        BGR(const Color& c):b(c.b), g(c.g), r(c.r){}
        Color toUt()const{ return {r,g,b,255}; }
        string str()const
        { stringstream s; s << b << "," << g << "," << r; 
          return s.str(); }
    }; 

    struct HSV {
        HSV(){}
        HSV(uint8_t h, uint8_t s, uint8_t v):
            h(h), s(s), v(v){}
        uint8_t h=0;
        uint8_t s=0;
        uint8_t v=0;  
        string str()const
        { stringstream ss; ss << (int)h << "," << (int)s << "," << (int)v; 
          return ss.str(); }
    };

    //------------
    // primitive
    //------------
    // TODO: template for Line 3d
    struct Line2d{
         Line2d(const vec2& p1, const vec2& p2):p1(p1), p2(p2){}
         Line2d(){ p1.Zero(); p2.Zero(); }
         vec2 p1; vec2 p2; 
         string str()const ;
         vec2 dv()const{ return p2-p1;}
         vec2 nv()const{ return dv().normalized(); }
         double len()const{ return dv().norm(); }
         double ang()const
         { vec2 v = dv();return atan2(v.y(), v.x()); }
         double dist(const vec2& v)const
         {  
            vec2 n = this->nv(); 
            vec2 vi = v - p1;
            double t = n.dot(vi);
            vec2 vp = p1 + t*n;
            return (v - vp).norm();
         }
         vec2 cntr()const{ return (p1+p2)*0.5; }
    };
    template<typename T>
        Line2d operator *(const T& m, const Line2d& l)
        { return Line2d(m*l.p1, m*l.p2); }
    inline Line2d operator + (const Line2d& l, const vec2& v)
    { return Line2d(l.p1 + v, l.p2 + v); }
    inline Line2d operator - (const Line2d& l, const vec2& v)
    { return Line2d(l.p1 - v, l.p2 - v); }
    //----
    struct Line{
         Line(const vec3& p1,
              const vec3& p2):p1(p1), p2(p2){}
         Line(){ p1.Zero(); p2.Zero(); }
         vec3 p1; vec3 p2; 
         string str()const ;
    };
    //-----
    struct Pose{ 
        quat q{1,0,0,0}; 
        vec3 t = zerov3(); 
        string str()const;
        Pose inv()const;
        vector<Line> axis(double l=1.0)const;
        Pose operator *(const Pose& p)const;
        vec3 operator *(const vec3& v)const;
    };
    //---- Cube
    struct Cube{
        Cube(){ c << 0,0,0; sz << 1,1,1; }
        vec3 c;
        vec3 sz;
        vec3s points()const;
        vector<Line> edges()const;
    };
    //---- 3d shape
    struct Cylinder{
        Pose pose;
        double r=1;
        double l=1;
        //--- vis
        int N_fan = 16;
        vec3s points()const;
        vector<Line> edges()const;
    };
    //---- Box3d
    struct Box3d{
        Rng<double> x,y,z;
        void upd(const vec3& v)
        { x.upd(v.x()); y.upd(v.y()); z.upd(v.z()); }
        vec3 min()const{ vec3 v; v << x.d0, y.d0, z.d0; return v; }
        vec3 max()const{ vec3 v; v << x.d1, y.d1, z.d1; return v; }
    };
    //---------
    // utils
    //---------
    //---- open CV wait key
    extern int cv_waitkey(int MS);
    extern bool cv_waitESC(int MS);
    extern void show_loop();
    //----
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
        static Sp<Img> create(); // factory

        virtual Sz size()const=0;
        virtual bool load(CStr& s, int cvFlag=1)=0;
        virtual bool save(CStr& s)const=0;
        virtual bool val()const =0;
        operator bool(){ return val(); }

        virtual void show(CStr& sWind)const=0;
        //---- TODO: template
        virtual void set(const Px& px, const Color& c)=0;
        virtual bool get(const Px& px, Color& c)const=0;
        virtual void set(const Px& px, const HSV& c)=0;
        virtual bool get(const Px& px, HSV& c)const=0;
        // note: return false if out of img dimention
        //---- draw functions, 
        virtual void draw(CStr& s, 
            const Px& px={30,30},
            const Color& c={255,255,255})=0;
        
        virtual void draw(const vec2s& vs, const Color& c, 
                          float w=1.0)=0;  
        virtual void draw(const vector<Line2d>& lns,
                          const Color& c={255,255,255}, 
                          double w=1.0)=0;
        //---- 2d draw functions
        virtual void draw(const ut::Rect& r, const Color& c, 
                          float w=1.0)=0;
        //--- 3d draw functions
        struct Axis{ Pose pose; double l=1; double w=1;};
        void draw(const CamCfg& cc, const Axis& a);
        void draw(const CamCfg& cc, const Cylinder& cl, const Color& c, float w=2);
         
        //---- processing
        virtual void filter(const HSV& c0,
                            const HSV& c1)=0;
        virtual void toGray()=0;
        virtual void toHsv()=0;
        //----
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

        //---- Hough line detection
        struct HoughLnCfg{
            double  	rho = 1; // pixel
            double  	theta = M_PI/180.0;
            int  	    TH = 150;
            double  	minLnLen = 100;
            double  	maxLnGap = 10;  
            //--- extra
            bool doCanny = true;           
        };
        virtual vector<Line2d> det(const HoughLnCfg& c)const=0;
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
    // InstSegm 
    //----------
    // Object instancing by color filter,
    //   Convex hull and mix of 
    //   OpenCV methods, without DNN.
    class InstSegm{
    public:
        struct Cfg{
            struct Filter{
                HSV c0,c1;
            }; Filter filter;
            float blurSz = 3;
            float areaTH = 20*10;
            bool en_imo = false;
            bool enShow = false;
        }; Cfg cfg_;
        //----
        struct Inst{
            vector<vec2> hull;
            ut::Rect box;
        };
        //----
        struct Data{
            vector<Inst> ins;
            Sp<Img> p_imo = nullptr;
             // contour img grayscale
            Sp<Img> p_imc = nullptr;
        }; Data data_;
        bool onImg(const Img& im);

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
            //---- marker board
            struct Board{
                struct Cfg{
                    string sName;
                    struct Mark{
                        int id=-1;
                        double w=1.0;
                        vec2 xy;
                    };
                    Box3d box;
                    vector<Mark> marks;
                    static Sp<Cfg> create();
                    virtual void init(int dict_id)=0;

                };
                //---- result
                Pose pose;
                Sp<Cfg> p_cfg = nullptr;
            };
            //---- Marker cfg
            struct MCfg{
                bool en_imo = false;
                string sDict_="aruco_dict_id0";
                int dict_id_=0;
                struct Grp{
                    set<int> ids;
                    double w=1;
                };
                vector<Grp> grps_;
                vector<Sp<Board::Cfg>> boards_;
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
            struct{ 
                vector<Marker> ms; 
                //--- result img with 
                // detect result draw.
                Sp<Img> p_imo = nullptr;
                //---- boards
                vector<Board> boards;
            }result_;
            //---- detect
            bool onImg(const Img& im);
        protected:
            Sp<Img> gen_imo(const Img& im)const;
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
        Points();
        struct Pnt{ vec3 p; Color c; };
        struct Data{};// virtual
        //----- visualization
        class Vis{
        public:
            struct Cfg{
                Cfg(){}
                string sName;
                Color bk_color;
                float axisL=10.0;
            }; 
            void add(const Points& ps, 
                     const string& sName,
                     float pnt_sz=3);
            bool spin();
            static Sp<Vis> create(const Cfg& c=Cfg());
            void clear();
        };
        //----
        void add(const Pnt& p);
        bool load(const string& sf);
        bool save(const string& sf)const;
        //--- filter statistical or voxel
        void filter_stats(float meanK = 50, float devTh = 1.0);
        void filter_voxel(float reso=0.03);

        auto getData(){ return p_data_ ; }
        auto getData()const{ return p_data_ ; }

        //---- samples
        void gen_cylinder();
    protected:
        Sp<Data> p_data_ = nullptr;
    };

    //------------
    // StereoVO
    //------------
    //Stereo video odometry
    class StereoVO{
    public:

        static Sp<StereoVO> create();
        //---- disparity cfg
        struct DisparityCfg{
            //---- SGBM cfg
            struct SGBM{
                int  	minDisparity = 0;
                int  	numDisparities = 16;
                int  	blockSize = 3;
                int  	P1 = 0;
                int  	P2 = 0;
                int  	disp12MaxDiff = 0;
                int  	preFilterCap = 0;
                int  	uniquenessRatio = 0;
                int  	speckleWindowSize = 0;
                int  	speckleRange = 0;
                //--- disparity WSL filter
                struct WLSFilter{
                    bool en=true;
                    float lambda=8000;
                    float sigma=1;
                }; WLSFilter wls_filter;
            }; SGBM sgbm;
            float vis_mul = 8.0;
        };
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

            DisparityCfg dispar;

            struct Run{
                bool bShow=false;
                bool enDense = false;
                bool enDepth = false;
                bool enWr = false;
            }; Run run;

            struct PointCloud{
                double z_TH = 40;
                struct Filter{
                    bool en = true;
                    float meanK = 50;
                    float devTh = 1.0;
                    float voxel_res = 0.03; 
                }; Filter filter;
            }; PointCloud pntCloud;

            //----
            bool load(const string& sf);
            string str()const;
        };
        Cfg cfg_;
        //---- Depth
        class Depth{
        public:
            //---- depth disparity map
            Sp<Img> p_imd_ = nullptr;
            //---- point cloud
            struct PntCloud{
                Sp<Points> p_dense  = nullptr;
                Sp<Points> p_sparse = nullptr;
            }; PntCloud pntc;
        };
        //---- Frm data
        struct Frm{
            // Triangulated feature points
            //  in global space.
            vec3s Pws; 
            //---- Depth
            Depth depth;
        };
        //----
        struct Data{
            int frmIdx = 0;
            //----
            struct Wr{
                ofstream ofs_pnts_spar;
                ofstream ofs_pnts_dense;
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

            //---- vis
            struct PntVis{

                Sp<Points::Vis> p_vis_dense = nullptr;
            }; PntVis pntVis;

            // wr data
            bool wrData();
            void close(){ wr.close(); }
        };
        //----
        virtual bool onImg(const Img& im1, 
                           const Img& im2)=0;

        virtual bool genDepth(const Img& im1,  
                              const Img& im2,
                              Depth& depth)=0;

        auto& getData()const{ return data_; }
        void onFinish(){ data_.close(); }
        void setFrmIdx(int i){ data_.frmIdx=i; }
        void showLoop();
    protected:

        Data data_;
    };

}

