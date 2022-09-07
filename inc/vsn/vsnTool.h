/*
   Author: Sherman Chen
   Create Time: 2022-07-07
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnLib.h"

using namespace vsn;

namespace app
{
    //-----------
    // util
    //-----------

    //-----------
    // CmdImg
    //-----------
    class CmdImg:public Cmd{
    public:
        using Cmd::Cmd;
        CmdImg();
        bool run_picker(CStrs& args);
    protected:
        struct Data{
            int frm_idx=0;
        }; Data data_;

    };
    //-----------
    // CmdVideo
    //-----------
    class CmdVideo:public Cmd{
    public:
        using Cmd::Cmd;
        CmdVideo();
        bool run_frames(CStrs& args);
    protected:
        struct Data{
            int frm_idx=0;
        }; Data data_;
        bool video_frm_ui(const Img& im);

    };
    //-----------
    // CmdMarker
    //-----------
    class CmdMarker:public Cmd{
    public:
        using Cmd::Cmd;
        CmdMarker();
        struct Cfg{
            CamCfg          camc;
            string swd; // write dir
            bool enShow = false;
            double rot=0;
            bool enWr = false;
            int skip_frm=0;
        };
        Cfg cfg_;
        //----
        Marker::PoseEstimator poseEstr_;
        //----
        bool run_det(CStrs& args)const;
        bool run_pose(CStrs& args);
        bool pose_est(Img& im, vector<Marker>& ms);
        void draw(Img& im,  
                const vector<Marker>& ms)const;
        bool run_pose_video(CStr& sf);
        bool run_pose_img(CStr& sf);
    };

    //-----------
    // VsnTool
    //-----------
    class VsnTool{
    public:
        bool run(CStrs& args);
    protected:
        Cmd cmd_;
        void initCmd(CStrs& args);
        
    };
}


