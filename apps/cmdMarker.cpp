#include "vsn/vsnTool.h"

using namespace app;

//----
CmdMarker::CmdMarker():
    Cmd("ArUco marker commands",
    [](CStrs& args){
        return true;
    })
{}

