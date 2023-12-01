#ifndef VOXEL_GRID_COUNTED_H
#define VOXEL_GRID_COUNTED_H

#include <pcl/filters/voxel_grid.h>
#include "vofod/point_types.h"

namespace vofod
{
  class VoxelGridCounted : public pcl::VoxelGrid<pcl::PointXYZI>
  {
  public:
    using PointT = pcl::PointXYZI;
    using PointCloudOut = pcl::PointCloud<vofod::PointXYZR>;

    VoxelGridCounted(const float threshold);
    void filter(PointCloudOut& output);
    void setVoxelAlign(const Eigen::Vector4f& align_center);

  private:
    void filterImpl(PointCloudOut& output);

    float threshold_;
    bool align_voxels_;
    Eigen::Vector4f align_center_ = Eigen::Vector4f::Zero();
  };
}

#endif // VOXEL_GRID_COUNTED_H
