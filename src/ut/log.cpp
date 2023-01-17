/*
   Author: Sherman Chen
   Create Time: 2023-01-17
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "ut/cutil.h"
#include <unistd.h>


namespace ut{

//-------------
// logf
//-------------
namespace utlog
{
    namespace{
        ofstream logFile_;
        FuncCbk f_cbk_ = nullptr;
    }
    
    //---- log file
    extern bool openFile(CStr& sFile)
    {
        logFile_.open(sFile);
        if(!logFile_.is_open())
        {
            err("Fail to open file:"+sFile);
            return false;
        }
        //----
        inf("log file open : "+sFile);
        return true;

    }
    extern void setCallbk(FuncCbk cbk)
    {
        f_cbk_ = cbk;
    }

    //---- base log
    void logs(CStr& s, bool isErr = false)
    {
        if(isErr)
            cerr << s << flush;
        else
            cout << s << flush;
        //----
        if(logFile_)
        {
            logFile_ << s;
            logFile_.flush();
        }

        //----
        if(f_cbk_!=nullptr)
            f_cbk_(s);
    }

    //-----
    extern void str(CStr& s)
    { logs(s); }
    extern void dbg(CStr& s)
    { logs("[dbg]:"+s +"\n"); }
    extern void inf(CStr& s)
    { logs("[info]:"+s +"\n"); }
    extern void err(CStr& s)
    { logs("[err]:"+s +"\n", true); }
    extern void errf(CStr& s)
    { logs("[err]: Failed to open file:"+s +"\n", true); }
}

} // ut
