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
        bool run_det(CStrs& args)const;

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


