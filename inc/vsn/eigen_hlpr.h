/*
   Author: Sherman Chen
   Create Time: 2022-05-16
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */
#pragma once
#include "vsn/cutil.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

namespace egn{
    using namespace ut;
    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;
    using vec5 = Eigen::Matrix<double,5,1>;

    using vec3s = vector<vec3>;
    using vec2s = vector<vec2>;
    extern string jstr(const vec2s& vs);

    using mat2 = Eigen::Matrix2d;
    using mat3 = Eigen::Matrix3d;
    using mat4 = Eigen::Matrix4d;
    using mat3x4 = Eigen::Matrix < double , 3 , 4 >;

    using quat = Eigen::Quaterniond;
    //--- vec initial
    inline vec2 zerov2(){ vec2 v; v << 0,0;   return v;}
    inline vec3 zerov3(){ vec3 v; v << 0,0,0; return v;}
    inline void init(vec2& v){ v << 0,0; }
    inline void init(vec3& v){ v << 0,0,0; }
    inline void init(quat& q){ q = quat(1,0,0,0); }
    inline vec3 nx3(){ vec3 v; v << 1,0,0; return v; }
    inline vec3 ny3(){ vec3 v; v << 0,1,0; return v; }
    inline vec3 nz3(){ vec3 v; v << 0,0,1; return v; }
    //-- rotation
    inline mat3 rotmat(const vec3& v, double rad)
    {  return Eigen::AngleAxisd(rad, v).matrix(); }
    //-- conv
    inline Px toPx(const vec2& v){ return {(int)v.x(), (int)v.y()}; };
    inline vec2 px2v(const Px& p){ vec2 v; v << p.x, p.y; return v; };
    inline bool normalize(const vec4& i, vec3& o)
    { 
      if(i(3)==0) return false;
      o << i(0)/i(3), i(1)/i(3), i(2)/i(3);
      return true;
    }
    //--- to/from string
    extern string str(double d, int deci=2);
    extern string str(const vec2& v, int deci=2);
    extern string str(const vec3& v, int deci=2);
    extern bool s2v(const string& s, vec2& v);
    extern bool s2v(const string& s, vec3& v);
    
    //--- utils
    extern vec2 avg(const vec2s& vs);
    extern vec3 avg(const vec3s& vs);
}
