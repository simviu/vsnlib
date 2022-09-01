#pragma once

#include "vsn/eigen_hlpr.h"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace pclu{
    using Pnt = pcl::PointXYZRGB;
    using PCloud = pcl::PointCloud<pcl::PointXYZRGB>;
}