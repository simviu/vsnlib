/*
   Author: Sherman Chen
   Create Time: 2023-01-30
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */


#include "vsn/vsnTool.h"
#include "vsn/vsnLibCv.h"

using namespace app;
using namespace Eigen;
//----
namespace{
    bool parse_data(const string& s, int n,
                    vector<double>& ds)
    {
        if(!s2data(s, ds, ','))
        {
            log_e("Fail to parse data:'"+s+"'");
            return false;
        }

        if(ds.size()!=n)
        {
            log_e("Expect number "+to_string(n)+
                ", got "+to_string(ds.size()));
            return false;
        }
        return true;
    }
}

//----
CmdMath::CmdMath()
{
    sHelp_ = "( math utils )";
    add("conv_rot", mkSp<Cmd>(
    "[euler|rvec|quat|R]=<DATA> (Convert to other values)",
    [&](CStrs& args)->bool{ return conv_rot(args); }));

}
//-------
bool CmdMath::conv_rot(CStrs& args)
{
    StrTbl kv; parseKV(args, kv);
    string sq = lookup(kv, string("quat"));
    string sr = lookup(kv, string("rvec"));
    string sR = lookup(kv, string("R"));
    string se = lookup(kv, string("euler"));

    quat q;
    Euler e;
    vec3 r;
    mat3 R; 
    vector<double> ds;
    if(sR!="")
    {
        if(!parse_data(sR, 9, ds)) return false;
        MatrixXd Rd = Map<Matrix<double, Dynamic, Dynamic, RowMajor>>
            (ds.data(), 3,3);
        R = Rd;
    }
    
    
    
    return true;
}

