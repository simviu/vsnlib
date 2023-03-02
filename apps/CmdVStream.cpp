/*
   Author: Sherman Chen
   Create Time: 2023-03-02
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */



#include "vsn/vsnTool.h"


using namespace app;

void CmdVStream::init_cmds()
{
    add("server", mkSp<vstream::Server>());
    add("client", mkSp<vstream::Client>());
}
