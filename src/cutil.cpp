/*
   Author: Sherman Chen
   Create Time: 2022-05-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/cutil.h"
#include <unistd.h>
#include <stdio.h>

#define PATH_BUF_LEN 1024

namespace ut{
    //--- file name/ path util
    namespace fn
    {
        extern string nopath(const string& s) 
        {
        // ref : https://btechgeeks.com/how-to-get-filename-from-a-path-with-or-without-extension-in-cpp/
            char sep = '/';
            #ifdef _WIN32
            sep = '\\';
            #endif
            size_t i = s.rfind(sep, s.length());
            if (i != string::npos) 
            {
                string filename = s.substr(i+1, s.length() - i);
                string rawname = filename.substr(0, s.length()); 
                return(rawname);
            }
            return("");
        }
    }
    //---- 
    extern bool parseKV(CStrs& ss, StrTbl& kv)
    {
        for(auto& s : ss)
        {
           if(s=="")continue;
           size_t e = s.find('=');
           //---- standalong string take as option
           if(e==string::npos)
           { kv[s] =""; continue; }
           //--- split k/v
           string sk = s.substr(0,e);
           string sv = s.substr(e+1);
           kv[sk] = sv;
        }
        return true;
    }

//-------------
// logf
//-------------
namespace utlog
{
    namespace{
        ofstream logFile_;
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

    //---- base log
    void logs(CStr& s, bool isErr = false)
    {
        if(isErr)
            cerr << s << endl << flush;
        else
            cout << s << endl << flush;
        //----
        if(logFile_)
        {
            logFile_ << (s + "\n");
            logFile_.flush();
        }
    }

    //-----
    extern void dbg(CStr& s)
    { logs("[dbg]:"+s); }
    extern void inf(CStr& s)
    { logs("[info]:"+s); }
    extern void err(CStr& s)
    { logs("[err]:"+s, true); }
    extern void errf(CStr& s)
    { logs("[err]: Failed to open file:"+s, true); }
}
//--------------------
// sys
//--------------------
namespace sys
{
    extern string pwd()
    {
        char s[PATH_BUF_LEN];
        s[0]=0;
        getcwd(s, sizeof(s));
        return string(s);
    }
    //-----
    void FPS::tick()
    {
        Time t = now();
        double dt = elapse(t_, t);
        t_ = t;
        //--- wait for 2nd tick()
        if(fps_<0) return;

        //---- sliding window average
        dts.push_back(dt);
        int n = dts.size();
        if(n > cfg_.N_avg)
        {
            dts.pop_front();
            n--;
        }

        //---- update
        double dt_sum = 0;
        for(auto& dt : dts)
            dt_sum += dt;

        fps_ = dt_sum / n;

    }

}

//--------------------
// file utils
//--------------------
bool fexist(CStr& sf)
{
    ifstream f;
    f.open(sf);
    bool ok = f.is_open();
    f.close();
    return ok;
}

//--------------------
// Test
//--------------------
bool Test::run()
{ 
    if(tests_.size()==0)
    {
        log_e("tests empty");
        return false; 
    }
    //----
    bool ok = true;
    
    for(auto it : tests_)
    {
        string s = it.first;
        ok &= run(s);
    }

    if(!ok)
         log_e("  tests has failure");
    else log_i(" tests passed");
    return ok;
}
//----
bool Test::run(const string& s)
{
    auto it = tests_.find(s);
    if(it==tests_.end())
    {
        log_e("  test not found:"+s);
        return false;
    }
    auto& t = *tests_[s];
    log_i("  ---- run test:'"+s+"'...");
    bool ok = t.run();
    if(ok) log_i("  ---- test '"+s+"' pass");
    else   log_e("  ---- test '"+s+"' fail");
    return ok;
}
//----
string Test::getTestsStr()const
{
    string s;
    for(auto it : tests_)
        s += it.first; s += " ";
    return s;
}
 
//--------------------
// Cmd
//--------------------
bool Cmd::run(CStrs& args)
{
    //--- run
    if(f_!=nullptr)
        return f_(args);

    // check subcmds
    if(args.size()==0) 
    {
        log_e("cmd function null and sub cmd not provided");
        return false;
    }
    //-------
    string sc = args[0];
    auto p = lookup(cmds_, sc);
    if(p==nullptr)
    {
        log_e("can't find subcmd:'"+sc+"'");
        return false;
        
    }
    //---- run subcmd
    Strs ss = args;
    ss.erase(ss.begin());
    return p->run(ss);

}


}// namespace ut
