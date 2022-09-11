/*
   Author: Sherman Chen
   Create Time: 2022-06-12
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/eigen_hlpr.h"

namespace egn
{
    namespace{
        void ss_deci(stringstream& s, int deci)
        {
            s.unsetf ( std::ios::floatfield );
            s.precision(deci); 
            s << std::fixed;
        }
    }

    //--- to strin
    extern string str(const vec2& v, int deci)
    {   
        stringstream s; ss_deci(s, deci); 
        s<< v.x() << ", " << v.y(); 
        return s.str(); 
    }
    extern string str(const vec3& v, int deci)   
    {   
        stringstream s; ss_deci(s, deci); 
        s<< v.x() << ", " << v.y() << ", " << v.z(); 
        return s.str(); 
    }
    //----
    template<class T>
    string jstr_vs(T vs)
    { 
        stringstream s; 
        for(int i=0;i<vs.size();i++)
        {
            if(i!=0) s<<", ";
            s << '"'<< vs[i] <<'"';
        }
        return s.str();
    }
    extern string jstr(const vec2s& vs)
    { return jstr_vs(vs); }
    extern string jstr(const vec3s& vs)
    { return jstr_vs(vs); }
    
    extern bool s2v(const string& s, vec2& v)
    {
        string s1 = s;
        std::replace(s1.begin(), s1.end(), ',', ' ');
        stringstream ss(s1);
        double x=0,y=0;
        ss >> x;
        ss >> y;
        v << x, y;
        return !(ss.fail());
    }
    extern bool s2v(const string& s, vec3& v)
    {
        
        string s1 = s;
        std::replace(s1.begin(), s1.end(), ',', ' ');
        stringstream ss(s1);
        double x=0,y=0,z=0;
        ss >> x;
        ss >> y;
        ss >> z;
        v << x, y, z;
        return !(ss.fail());
        

    }
    
    extern vec2 avg(const vec2s& vs)
    {
        vec2 v; v << 0,0;
        int N = vs.size();
        if(N==0) return v;
        for(auto& vi : vs)
            v += vi;
        v *= 1.0/N;
        return v;
    }
    extern vec3 avg(const vec3s& vs)
    {
        vec3 v; v << 0,0,0;
        int N = vs.size();
        if(N==0) return v;
        for(auto& vi : vs)
            v += vi;
        v *= 1.0/N;
        return v;  
    }

}