#ifndef VOXEL_GRID_WEIGHTED_H
#define VOXEL_GRID_WEIGHTED_H

#include <pcl/filters/voxel_grid.h>
#include "vofod/point_types.h"

namespace vofod
{
  class VoxelGridWeighted : public pcl::VoxelGrid<ouster_ros::Point>
  {
    using PointT = ouster_ros::Point;
    using PointCloudOut = pcl::PointCloud<vofod::PointXYZR>;
    void filterImpl(PointCloudOut& output);
  public:
    VoxelGridWeighted() {filter_name_ = "VoxelGridWeighted";}
    void filter(PointCloudOut& output);
    void setVoxelAlign(const Eigen::Vector4f& align_center);
  private:
    bool align_voxels_;
    Eigen::Vector4f align_center_ = Eigen::Vector4f::Zero();
  };
}

#endif // VOXEL_GRID_WEIGHTED_H
