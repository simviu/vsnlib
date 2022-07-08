#include "vsn/vsnTool.h"

using namespace app;

//----
CmdMarker::CmdMarker():
    Cmd("ArUco marker commands")
{
    //---- 'det'
    add("det", mkSp<Cmd>("detect marker on image or video",
    [&](CStrs& args)->bool{ return run_det(args); }));

}
//----
bool CmdMarker::run_det(CStrs& args)const
{
    StrTbl kv;
    parseKV(args, kv);
    string sf = lookup(kv, string("file"));
    auto p = Img::create();
    if(!p->load(sf))
        return false;
    vector<Marker> ms;
    Marker::detect(*p, ms);
    stringstream ss;
    for(auto& m : ms)
        ss << "Found marker:" << m.str() << endl;
    log_i(ss.str());
    return true;
}

