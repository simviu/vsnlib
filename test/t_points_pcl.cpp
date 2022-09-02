/*
   Author: Sherman Chen
   Create Time: 2022-09-01
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
//-----------
namespace{
    //---- RGB vis
    pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
    {
     
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0.7, 0.7, 1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }
    //----------------------
    void gen_cylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr)
    {
        std::uint8_t r(255), g(15), b(15);
        for (float z(-1.0); z <= 1.0; z += 0.05)
        {
            for (float angle(0.0); angle <= 360.0; angle += 5.0)
            {
                pcl::PointXYZRGB point;
                point.x = 0.5 * std::cos (pcl::deg2rad(angle));
                point.y = sinf (pcl::deg2rad(angle));
                point.z = z;
                std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                        static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
                point.rgb = *reinterpret_cast<float*>(&rgb);
                point_cloud_ptr->points.push_back (point);
            }
            if (z < 0.0)
            { r -= 12;  g += 12; }
            else
            { g -= 12;  b += 12; }
        }
        point_cloud_ptr->width = point_cloud_ptr->size ();
        point_cloud_ptr->height = 1;

    }

}
//-------------
bool TestPoints::test_pcl_wr()
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
    return true;
}

//-------------
// ref : https://pcl.readthedocs.io/projects/tutorials/en/latest/pcl_visualizer.html
bool TestPoints::test_pcl_vis()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    gen_cylinder(point_cloud_ptr);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = rgbVis(point_cloud_ptr);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
    return true;
}
