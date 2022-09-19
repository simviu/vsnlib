/*
   Author: Sherman Chen
   Create Time: 2022-07-07
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTool.h"

using namespace vsn;
using namespace ut;
using namespace app;
//---------
// initCmd
//---------
void VsnTool::initCmd(CStrs& args)
{
    //----
    cmd_.add("calib",   mkSp<CmdCalib>());
    cmd_.add("marker",  mkSp<CmdMarker>());
    cmd_.add("image",   mkSp<CmdImg>());
    cmd_.add("video",   mkSp<CmdVideo>());
}

//---------
// run
//--------- 
bool VsnTool::run(CStrs& args)
{
    log_i("-- run VsnTool --");
    log_i("cur dir:"+sys::pwd());

    //---- init cmd
    initCmd(args);

    //--- run
    cmd_.run(args);

    return true;
}




//----------
// main
//----------
int main(int argc, char ** argv)
{
    log_i("---- run "+string(argv[0]));
    VsnTool tool;
    Strs args;
    for(int i=1;i<argc;i++)
        args.push_back(argv[i]);
    bool ok = tool.run(args);
    return ok?0:1;
}
