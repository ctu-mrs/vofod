#include "vofod/voxel_grid_weighted.h"

namespace vofod
{

  struct cloud_point_index_idx2
  {
    unsigned int idx;
    unsigned int cloud_point_index;
    int ijk0, ijk1, ijk2;

    cloud_point_index_idx2(unsigned int idx_, unsigned int cloud_point_index_, int ijk0, int ijk1, int ijk2)
      : idx (idx_), cloud_point_index (cloud_point_index_), ijk0(ijk0), ijk1(ijk1), ijk2(ijk2)
    {}

    bool operator<(const cloud_point_index_idx2 &p) const
    {
      return (idx < p.idx);
    }
  };

  void VoxelGridWeighted::setVoxelAlign(const Eigen::Vector4f& align_center)
  {
    align_voxels_ = true;
    align_center_ = align_center;
  }

  void VoxelGridWeighted::filter(PointCloudOut& output)
  {
    if (!initCompute ())
      return;

    output.header = input_->header;
    output.sensor_origin_ = input_->sensor_origin_;
    output.sensor_orientation_ = input_->sensor_orientation_;
    filterImpl(output);

    deinitCompute ();
  }

  void VoxelGridWeighted::filterImpl(PointCloudOut& output)
  {
    // Has the input dataset been set already?
    if (!input_)
    {
      PCL_WARN ("[pcl::%s::filterImpl] No input dataset given!\n", getClassName ().c_str ());
      output.width = output.height = 0;
      output.points.clear ();
      return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height       = 1;                    // downsampling breaks the organized structure
    output.is_dense     = true;                 // we filter out invalid points

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    pcl::getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    const int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+2;
    const int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+2;
    const int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+2;

    if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
      PCL_WARN("[pcl::%s::filterImpl] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
      return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

    // Compute the voxelmap's coordinate offset
    Eigen::Vector4f offset = min_b_.cast<float>().cwiseProduct(leaf_size_);
    if (align_voxels_)
    {
      Eigen::Vector4f align_corner_offset = Eigen::Vector4f::Zero();

      // calculate the offset to align the voxel corners with the desired grid
      align_corner_offset[0] = fmod(align_center_[0] - leaf_size_[0]/2, leaf_size_[0]);
      align_corner_offset[1] = fmod(align_center_[1] - leaf_size_[1]/2, leaf_size_[1]);
      align_corner_offset[2] = fmod(align_center_[2] - leaf_size_[2]/2, leaf_size_[2]);

      // because all points in the input dataset have to fit within the new bounds,
      // shift the voxel grid in the negative direction and then recalculate min_b_
      if (align_corner_offset[0] < 0)
        align_corner_offset[0] += leaf_size_[0];
      if (align_corner_offset[1] < 0)
        align_corner_offset[1] += leaf_size_[1];
      if (align_corner_offset[2] < 0)
        align_corner_offset[2] += leaf_size_[2];

      // apply the corner alignment offset
      offset -= align_corner_offset;

      // recalculate min_b_ to accomodate the shifted and possibly enlarged grid
      min_b_[0] = static_cast<int> (floor (offset[0] * inverse_leaf_size_[0]));
      min_b_[1] = static_cast<int> (floor (offset[1] * inverse_leaf_size_[1]));
      min_b_[2] = static_cast<int> (floor (offset[2] * inverse_leaf_size_[2]));
    }

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Storage for mapping leaf and pointcloud indexes
    std::vector<cloud_point_index_idx2> index_vector;
    index_vector.reserve (indices_->size ());

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!std::isfinite (input_->points[*it].x) || 
            !std::isfinite (input_->points[*it].y) || 
            !std::isfinite (input_->points[*it].z))
          continue;

      const int ijk0 = static_cast<int> (floor ( (input_->points[*it].x - offset[0]) * inverse_leaf_size_[0] ));
      const int ijk1 = static_cast<int> (floor ( (input_->points[*it].y - offset[1]) * inverse_leaf_size_[1] ));
      const int ijk2 = static_cast<int> (floor ( (input_->points[*it].z - offset[2]) * inverse_leaf_size_[2] ));

      // Compute the centroid leaf index
      const int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      const cloud_point_index_idx2 cpidx(static_cast<unsigned int> (idx), *it, ijk0, ijk1, ijk2);
      index_vector.push_back(cpidx);
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx2>());

    // Third pass: count output cells
    // we need to skip all the same, adjacenent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve (index_vector.size ());
    while (index < index_vector.size ()) 
    {
      unsigned int i = index + 1;
      while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
        ++i;
      if (i - index >= min_points_per_voxel_)
      {
        ++total;
        first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
      }
      index = i;
    }

    // Fourth pass: insert occupied voxel centers into the returned container together with their weight (number of points within)
    output.points.resize (total);
    index = 0;
    for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp)
    {
      // calculate the corresponding voxel center
      const unsigned int first_index = first_and_last_indices_vector[cp].first;
      const unsigned int last_index = first_and_last_indices_vector[cp].second;

      const auto& cpidx = index_vector[first_index];
      const float x = (static_cast<float>(cpidx.ijk0) + 0.5f) * leaf_size_[0] + offset[0];
      const float y = (static_cast<float>(cpidx.ijk1) + 0.5f) * leaf_size_[1] + offset[1];
      const float z = (static_cast<float>(cpidx.ijk2) + 0.5f) * leaf_size_[2] + offset[2];
      const float weight = static_cast<float>(last_index - first_index);
      output.points[index].x = x;
      output.points[index].y = y;
      output.points[index].z = z;
      output.points[index].intensity = weight;

      ++index;
    }
    output.width = static_cast<uint32_t> (output.points.size ());
  }

}
