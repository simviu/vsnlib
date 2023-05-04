/*
   Author: Sherman Chen
   Create Time: 2023-05-04
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */



#include "vsn/vsnTool.h"
#include "vsn/vsnLibCv.h"

using namespace app;
using namespace vsn;
//----
namespace{
}

//----
CmdPoints::CmdPoints()
{
    sHelp_ = "(Point cloud tools)";

    //----
    add("show", mkSp<Cmd>("file=<FILE> name=<NAME> pnt_sz=<SIZE>",
    [&](CStrs& args)->bool{ return show(args); }));
}

//----

bool CmdPoints::show(CStrs& args)
{
    KeyVals kvs(args);
    string sf = kvs["file"];

    Points pnts;
    if(!pnts.load(sf)) return false;
    Points::Vis::Cfg c;
    c.axisL = 1; // TODO: temp dbg
    if(!c.set(kvs))return false;


    auto pv = Points::Vis::create(c);
    double pnt_sz=0.3; 

    if(!kvs.get("pnt_sz", pnt_sz))return false;
    string sName = kvs["name"];
    pv->add(pnts, sName, pnt_sz);

    //----
    while(1)
    {
        pv->spin();
        sys::sleep(0.01);
    }
    return true;
}

