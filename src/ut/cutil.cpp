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
            if(t!="")
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



}// namespace ut
