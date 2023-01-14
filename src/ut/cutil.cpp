/*
   Author: Sherman Chen
   Create Time: 2022-05-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "ut/cutil.h"
#include <unistd.h>
#include <stdio.h>
#include <filesystem>

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
    string remove(const string& s, const char c)
    {
        string s1 = s;
        s1.erase(remove(s1.begin(), s1.end(), c), s1.end()); 
        return s1;
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
    //----
    extern vector<string> tokens(
        const string& s, char c_deli)
    {
        vector<string> ts;
        stringstream ss(s);
        string t;
        while(getline(ss, t, c_deli))
            ts.push_back(t);
        
        return ts;
    }
    //-----
    extern bool s2data(const string& s, vector<double>& ds, char c_deli)
    {
        auto ts = tokens(s, c_deli);
        try
        {
            for(auto& t : ts)
                ds.push_back(stod(t));
        }
        catch(exception& e)
        {  
            return false; 
        }
        return true;
    }
    //-----
    extern bool s2data(const string& s, vector<int>& ds, char c_deli)
    {
        auto ts = tokens(s, c_deli);
        try
        {
            for(auto& t : ts)
                ds.push_back(stoi(t));
        }
        catch(exception& e)
        {  
            return false; 
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
            cerr << s << flush;
        else
            cout << s << flush;
        //----
        if(logFile_)
        {
            logFile_ << s;
            logFile_.flush();
        }
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
//--------------------
// sys
//--------------------
namespace sys
{

    //-----
    FPath::FPath(const string& sf)
    {
        std::filesystem::path p(sf);
        path = string(p.parent_path()) + "/";
        base = p.stem();
        ext = p.extension();
    }

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
        if(fps_<0) 
        { fps_ =0; return; }

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
        if(dt_sum==0) return;
        fps_ = n / dt_sum ;

    }
    //----
    extern bool exists(const string& s)
    {
        return filesystem::exists(s);
    }

}
//--------------------
// Elements
//--------------------
bool Color::set(const string& s, char c_deli)
{
    vector<int> ds;
    if( (!s2data(s, ds, c_deli)) || ds.size()<3 )
        return false;
    r = ds[0]; g=ds[1]; b=ds[2];
    if(ds.size()>3) a=ds[3];
    return true; 
}

//----
bool Px::set(const string& s, char c_deli)
{   
    vector<double> ds;
    if( (!s2data(s, ds, c_deli)) ||
        ds.size()<2 )
        return false;
    x = ds[0]; y=ds[1]; 
    return true; 
}

//-----
bool Sz::set(const string& s, char c_deli)
{
    vector<int> ds;
    if( (!s2data(s, ds, c_deli)) ||
        ds.size()<2 )
        return false;
    w = ds[0]; h=ds[1]; 
    return true; 

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
bool Cmd::run(const string& sLn)
{
    return run(tokens(sLn, ' '));
}

//--------
bool Cmd::run(int argc, char ** argv)
{
    //---- Check console mode
    if(argc==1)
        return run_console();

    //---- run with args
    Strs args;
    for(int i=1;i<argc;i++)
        args.push_back(argv[i]);
    
    //--- check script -f
    if(args[0]=="-f")
    {
        if(args.size()<2)
        {
            log_e("Expect <FILE> after '-f'");
            return false;
        }
        //----
        return runFile(args[1]);
    }

    //---- Normal run
    bool ok = run(args);
    return ok;
}
//----
bool Cmd::runFile(CStr& sf)
{
    log_i("Cmd::runFile() :'"+sf+"'...");
    ifstream f;
    f.open(sf);
    if(!f.is_open())
    {
        log_ef(sf);
        return false;
    }
    //
    int i=0;
    bool ok = true;
    while(!f.eof())
    {
        i++;
        string s;
        getline(f, s);
        if(s=="")continue;

        ok = run(s);
        if(!ok)
        {
            log_e("Line "+to_string(i)+" in '"+sf+"':");
            log_e("  Cmd fail:'"+s+"'");
            break;
        }
    }
    log_e("Cmd::runFile() done");
    return true;

}

//----
bool Cmd::run_console()
{
    log_i("Cmd console, 'help' for help, 'quit' to exit\n");

    while(1)
    {
        log_s("> ") ;
        string sln;
        std::getline(std::cin, sln);
        if(sln=="")continue;
        
        //--- check quit
        if(sln=="quit") break;

        //--- run
        auto args = tokens(sln, ' ');
        run(args);
    }
    return true;
}
//-----
bool Cmd::run(CStrs& args)
{
    //--- run
    if(f_!=nullptr)
        return f_(args);

    // check subcmds
    if(cmds_.size()==0) 
    {
        log_e("Cmd not init and no subcmds");
        return false;
    }
    //-------
    string sc = args[0];
    if(sc=="help")
    {
        std::cout << help() << std::endl;
        return true;

    }
    
    //----
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
//-----
string Cmd::help(const string& s_prefix)const
{
    string s= s_prefix + " "+ sHelp_ + "\n";
    for(auto& it : cmds_)
    {
        auto& sCmd = it.first;
        auto p = it.second;
        s += p->help("  "+s_prefix + " " + sCmd);        
    }
   
    return s;
}



}// namespace ut
