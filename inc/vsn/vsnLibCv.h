#include "vsnLib.h"
#include "ocv_hlpr.h"


namespace vsn
{


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


        virtual bool load(ut::CStr& s, int cvFlag) override;
        virtual bool save(ut::CStr& s) override;
        virtual bool val()const override
        { return (im_.rows>0) && (im_.cols>0);}
        virtual void show(CStr& sWind)const override;

        virtual void draw(CStr& s, 
            const Px& px={30,30},
            const Color& c={255,255,255})override;
       virtual void line(const Line2d& l,
                          const Color& c, 
                          double w=1.0)override;
        virtual void draw(const CamCfg& cc, const Axis& a)override;
        virtual void draw(const ut::Rect& r, const Color& c, 
                          float w=1.0)override;

        virtual void toGray()override;
        virtual void toHsv()override;
        virtual void filter(const Color& c0,
                            const Color& c1) override;
        //-----
        virtual void* data()override
        { return reinterpret_cast<void*>(&(im_)); }
        virtual const void* data()const override
        { return reinterpret_cast<const void*>(&(im_)); }
        virtual Sp<Img> copy()const override
        {  auto p = mkSp<ImgCv>(); 
           im_.copyTo(p->im_); return p;  }
        virtual void rot(double dgr)override;

        //---- dict selection ref OpenCV ArUco.
        // default is 5x5_250, TODO: change dict
     //   virtual void detect(vector<vsn::Marker>& markers)override;

        virtual void undistort(const CamCfg& cc)override;
        cv::Mat im_;

    protected:
    };
    //---- cast utils
    //------------
    // VideoCv
    //------------
    // Implementation of Video
    class VideoCv : public vsn::Video{
    public:
        VideoCv(){};
        VideoCv(CStr& s);
        virtual Sp<Img> read()override;
        
        bool isOpen() { return cap_.isOpened(); }
        bool createWr(CStr& sf);
        virtual bool write(const Img& im)override;
        virtual void close()override;

    protected:
        cv::VideoCapture cap_;
        Sp<cv::VideoWriter> p_vwr = nullptr;

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
    
    //------------
    // StereoVOcv
    //------------
    //Stereo video odometry
    class StereoVOcv : public StereoVO{
    public:
        //--- Triangulation 3d pnt of
        // Stereo match pnt
        struct MPnt{
            int mi=-1; // index to matches
            cv::Point3f Pt; // 3d triangulation pnt
            cv::Point3f Pd; // 3d pnt by depth
        };

        //--- frm data
        struct Frm{
            Sp<FeatureMatchCv> p_fm = nullptr;
            // 3d triangulation of matched feature points.
            // (size of mpnts same as matched feature pairs)
            vector<MPnt> mpnts; 

            //--- inliers mi set after solving 2d/3d
            //set<int> inliers;
            //--- find mpnt by feature index
            bool find(int i, bool bLeft, MPnt& mpnt)const;
            bool at(int mi, MPnt& mpnt)const;
        };
        //--- cv data
        struct Data{
            //---- previous feature match
            Sp<Frm> p_frm_prev = nullptr;
        };
        Data data_;
        //----
        virtual bool onImg(const Img& im1, 
                           const Img& im2)override;

        virtual bool genDepth(const Img& im1,  
                              const Img& im2)override;
    protected:
        bool odometry(const Frm& frm1,
                      const Frm& frm2);
        bool solve_2d3d(const Frm& frm1,
                        const Frm& frm2,
                        bool bLeft,
                        cv::Mat& r, cv::Mat& t,
                        set<int>& inliers)const;
        bool triangulate(const FeatureMatchCv& fm,
                         vector<MPnt>& mpnts)const;
        void calc_pnts(const Frm& frmc,
                       const set<int>& mi_ary,
                       vec3s& Ps)const;
        bool genDense();
    };
}
