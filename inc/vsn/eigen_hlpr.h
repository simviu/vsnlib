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

namespace egn{
    using namespace ut;
    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;
    using vec5 = Eigen::Matrix<double,5,1>;

    using vec3s = vector<vec3>;
    using vec2s = vector<vec2>;
    extern string jstr(const vec2s& vs);

    using mat3 = Eigen::Matrix3d;
    using mat4 = Eigen::Matrix4d;
    using mat3x4 = Eigen::Matrix < double , 3 , 4 >;

    using quat = Eigen::Quaterniond;
    //-- conv
    inline Px toPx(const vec2& v){ return {(int)v.x(), (int)v.y()}; };
    inline bool normalize(const vec4& i, vec3& o)
    { 
      if(i(3)==0) return false;
      o << i(0)/i(3), i(1)/i(3), i(2)/i(3);
      return true;
    }
    //--- to strin
    extern string str(const vec2& v, int deci=2);
    extern string str(const vec3& v, int deci=2);
        
}
