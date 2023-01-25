#include "vsn/vsnLib.h"



using namespace vsn;
using namespace stereo;
//----
namespace{
    using OmniCfg = VO::Cfg::Omni;
    //---- impl
    struct OmniCfgImp : public OmniCfg{
        bool loadFile(const string& sf)
        {
            return true;
        }
    protected:
    };
}


//----------------
//---- factory
Sp<OmniCfg> OmniCfg::load(const string& sf)
{  
    auto p = mkSp<OmniCfgImp>();
    if(p->loadFile(sf)) return p;
    return nullptr; 
} 


