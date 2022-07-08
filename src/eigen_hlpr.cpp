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
}