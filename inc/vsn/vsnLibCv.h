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
        ImgCv(cv::Mat im):im_(im){};
        ImgCv(const Img& im)
        {  
            im_ = *reinterpret_cast<const cv::Mat*>(im.data());
        }

        virtual bool load(ut::CStr& s, int cvFlag) override;
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
        virtual void toGray()override;
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
        bool createWr(CStr& sf, const Cfg& cfg);
        virtual bool write(const Img& im)override;

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
    // StereoVO_cv
    //------------
    //Stereo video odometry
    class StereoVOcv : public StereoVO{
    public:
        //--- frm data
        struct Frm{
            Sp<FeatureMatchCv> p_fm = nullptr;
            vec3s Ps; // Local triangulation points.

        };
        //--- Stereo match pnt
        struct SPnt{
            //--- index of feature match pnt
            // on L/R img 
            int fi1=-1; // -1 as invalid flag
            int fi2=-1;
            vec3 P; // 3d triangulation pnt
        };
        //--- cv data
        struct Data{
            //---- previous feature match
            Sp<Frm> p_frm_prev = nullptr;
            //---- L/R matched feature pnt
            vector<SPnt> sps;
        };
        Data data_;
        //----
        virtual bool onImg(const Img& im1, 
                           const Img& im2)override;

        virtual bool genDepth(const Img& im1,  
                              const Img& im2)override;
    protected:
        bool odometry(const Frm& frm1,
                      const Frm& frm2)const;
    };
}
