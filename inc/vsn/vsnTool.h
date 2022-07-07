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
    // CmdMarker
    //-----------
    class CmdMarker:public Cmd{
    public:
        using Cmd::Cmd;
        CmdMarker();
     //   virtual bool run(CStrs& args)override;
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


