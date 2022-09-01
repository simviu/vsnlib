/*
   Author: Sherman Chen
   Create Time: 2022-07-05
   Email: schen@simviu.com
   Copyright(c): Simviu Inc.
   Website: https://www.simviu.com
 */

#include "vsn/vsnTest.h"
#include "vsn/vsnLibCv.h"
#include "vsn/pcl_utils.h"

using namespace vsn;
using namespace ut;
using namespace test;

namespace{
    const struct{


    }lc_;
    //----
    // basic PCL test
    void test_pcl_wr()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Fill in the cloud data
        cloud.width    = 5;
        cloud.height   = 1;
        cloud.is_dense = false;
        cloud.points.resize (cloud.width * cloud.height);

        for (auto& point: cloud)
        {
          point.x = 1024 * rand () / (RAND_MAX + 1.0f);
          point.y = 1024 * rand () / (RAND_MAX + 1.0f);
          point.z = 1024 * rand () / (RAND_MAX + 1.0f);
        }

        pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    }

}
//--------------------------
bool TestPoints::run()
{
    // test basic
    test_pcl_wr();
    
    return true;
}
