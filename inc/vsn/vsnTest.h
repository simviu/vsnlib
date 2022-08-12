/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */
#pragma once

#include "vsn/vsnLib.h"

using namespace vsn;

namespace test{
    //------
    class TestMarker : public Test
    {
    public:
        virtual bool run() override;
    };
    //------
    class TestFeature : public Test
    {
    public:
        virtual bool run() override;
    };
}


