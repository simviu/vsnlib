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

    //---- TODO: deprecated and put in KeyVals
    extern bool parseKV(CStrs& ss, StrTbl& kv)
    {
        for(auto& si : ss)
        {
            string s = si;
            s.erase(remove(s.begin(), s.end(), ' '), s.end());

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

    //-------------- -----
    //-----
    bool KeyVals::parse(const string& s, char c_sep)
    { 
        string s1 = s;
        std::replace(s1.begin(), s1.end(), '\n', c_sep);         
        return parse(tokens(s1, c_sep)); 
    }

    //----
    bool KeyVals::has(const string& skey)const
    {
        return items.find(skey)!=items.end();
    }
    string KeyVals::get(const string& skey)const
    {
        auto it = items.find(skey);
        if(it==items.end()) {
            log_e("  key '"+skey+"' not found");
            return "";
        }
        return it->second;
    }
    //--------
    //------
    namespace{
        //----- Help func for KeyVals::get()
        template<typename T> bool get_val(
            const KeyVals& kvs, 
            const string& skey,
            const string& sType,
            T& d)
        {
            string sv = kvs.get(skey);
            if(sv=="") return false;
            if(s2d<T>(sv, d)) return true; 
            
            log_e("  For key '"+skey+"' fail to parse '"+
                    sv+"' to "+sType);
            return false;             
        }
    }
    //--------
    bool KeyVals::get(const string& skey, double& d)const
        {  return get_val(*this, skey, "'double'", d); }
    bool KeyVals::get(const string& skey, int& d)const
        {  return get_val(*this, skey, "'int'", d); }
    bool KeyVals::get(const string& skey, bool& d)const
        {  return get_val(*this, skey, "'bool'", d); }
    //--------
    bool KeyVals::get(const string& skey, string& s)const
    {   
        auto it = items.find(skey);
        if(it==items.end()) {
            log_e("  key '"+skey+"' not found");
            return false;
        }
        s = it->second;
        return true;
    }

    //-----------------
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

//-----
extern bool s2hex(const string& s, uint32_t& d)
{
    if( s.length()<2) return false;
    if( (s[0]!='0') || (s[1]!='x') ) return false;
    string sx = s.substr(2, s.length()-2);
    stringstream ss;
    ss >> std::hex >> d;
    return (!ss.fail()); 
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
    //----
    extern bool mkdir(const string& s)
    {
        if(exists(s))return true;
        bool ok = std::filesystem::create_directory(s);
        if(ok) log_i("mkdir:'"+s+"'");
        else log_e("fail to mkdir:'"+s+"'");
        return ok;
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
/*
bool fexist(CStr& sf)
{
    ifstream f;
    f.open(sf);
    bool ok = f.is_open();
    f.close();
    return ok;
}
*/


}// namespace ut
