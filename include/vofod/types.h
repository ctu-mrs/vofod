#pragma once

#include "vofod/point_types.h"

namespace vofod
{
  using pt_t = ouster_ros::Point;
  using pc_t = pcl::PointCloud<pt_t>;
  
  using pt_XYZ_t = pcl::PointXYZ;
  using pc_XYZ_t = pcl::PointCloud<pt_XYZ_t>;
  using pt_XYZR_t = vofod::PointXYZR;
  using pc_XYZR_t = pcl::PointCloud<pt_XYZR_t>;
  using pt_XYZI_t = pcl::PointXYZI;
  using pc_XYZI_t = pcl::PointCloud<pt_XYZI_t>;
  using pt_XYZRI_t = vofod::PointXYZRI;
  using pc_XYZRI_t = pcl::PointCloud<pt_XYZRI_t>;
  using pt_XYZt_t = pcl::PointXYZI;
  using pc_XYZt_t = pcl::PointCloud<pt_XYZt_t>;
  using octree = pcl::octree::OctreePointCloudSearch<pt_XYZ_t>;
  
  using vec2_t = Eigen::Vector2d;
  using vec3_t = Eigen::Vector3f;
  using vec3i_t = Eigen::Vector3i;
  using vec4_t = Eigen::Vector4f;
  using quat_t = Eigen::Quaternionf;
  using anax_t = Eigen::AngleAxisf;
  using mat3_t = Eigen::Matrix3f;
  using vec3s_t = Eigen::Matrix<float, 3, -1>;
}
