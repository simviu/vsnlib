#pragma once

#include "vsn/eigen_hlpr.h"


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace pclu{
    using Pnt = pcl::PointXYZRGB;
    using PCloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using ConstPCloud = const pcl::PointCloud<pcl::PointXYZRGB>;
}