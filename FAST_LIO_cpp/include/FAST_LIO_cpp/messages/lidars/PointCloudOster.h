#pragma once

#include <pcl/point_cloud.h>
#include <FAST_LIO_cpp/messages/lidars/PointOuster.h>

namespace fast_lio {

    using PointCloudOuster = pcl::PointCloud<PointOuster>;

} // namespace fast_lio
