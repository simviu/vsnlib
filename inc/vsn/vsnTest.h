/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
 */
#pragma once

#include "vsn/vsnLib.h"

using namespace vsn;

namespace test{
    class TestMarker : public Test
    {
    public:
        virtual bool run() override;
    };
}


