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
    class TestInst : public Test
    {
    public:
        virtual bool run() override;
    };
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
    }; //------
    class TestKittiStereo : public Test
    {
    public:
        virtual bool run() override;
    };
    //------
    class TestBlob : public Test
    {
    public:
        virtual bool run() override;
    };
     //------
    class TestPoints : public Test
    {
    public:
        virtual bool run() override;
    protected:
        bool test_pcl_wr();
        bool test_pcl_vis();
    };
}


