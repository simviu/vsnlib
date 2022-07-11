/*
   Author: Sherman Chen
   Create Time: 2022-07-07
   Email: schen@simviu.com
 */

#include "vsn/vsnLib.h"

using namespace vsn;

namespace app
{
    //-----------
    // util
    //-----------

    //-----------
    // CmdMarker
    //-----------
    class CmdMarker:public Cmd{
    public:
        using Cmd::Cmd;
        CmdMarker();
    protected:
        struct Cfg{
            Marker::Cfg     mcfg;
            CamCfg          camc;
            string swd; // write dir
        };
        Cfg cfg_;
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


