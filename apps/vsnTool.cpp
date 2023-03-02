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
void VsnTool::initCmd()
{
    //----
    add("calib",   mkSp<CmdCalib>());
    add("marker",  mkSp<CmdMarker>());
    add("image",   mkSp<CmdImg>());
    add("video",   mkSp<CmdVideo>());
    add("math",      mkSp<CmdMath>());
    add("vstream",   mkSp<CmdVStream>());
}





//----------
// main
//----------
int main(int argc, char ** argv)
{

    log_i("-- run VsnTool --");
    log_i("cur dir:"+sys::pwd());

    VsnTool tool;
    return tool.run(argc, argv);
}
