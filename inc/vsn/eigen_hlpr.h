/*
   Author: Sherman Chen
   Create Time: 2022-05-16
   Email: schen@simviu.com
 */
#pragma once
#include "vsn/cutil.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace egn{
    using namespace ut;
    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;
    using vec5 = Eigen::Matrix<double,5,1>;

    using mat3 = Eigen::Matrix3d;
    using mat4 = Eigen::Matrix4d;
    using mat3x4 = Eigen::Matrix < double , 3 , 4 >;

    using quat = Eigen::Quaterniond;
    //-- conv
    inline Px toPx(const vec2& v){ return {(int)v.x(), (int)v.y()}; };

    //--- to strin
    extern string str(const vec2& v, int deci=2);
    extern string str(const vec3& v, int deci=2);
        
}
