#pragma once

#include <pcl/point_cloud.h>
#include <FAST_LIO_cpp/messages/lidars/PointVelodyne.h>

namespace fast_lio {

    using PointCloudVelodyne = pcl::PointCloud<PointVelodyne>;

} // namespace fast_lio
