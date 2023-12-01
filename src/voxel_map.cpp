#include "vofod/voxel_map.h"
#include <cmath>
#include <cassert>

using namespace vofod;

VoxelMap::VoxelMap() : m_size_x(0), m_size_y(0), m_size_z(0) {};

/* resize() method overloads //{ */

void VoxelMap::resize(const vec3_t& center, const vec3_t& dimensions, const coord_t voxel_size)
{
  const coord_t voxel_size_inv = coord_t(1) / voxel_size;

  const vec3_t offset = center - dimensions / coord_t(2);
  const vec3i_t sizes = (voxel_size_inv * dimensions).array().ceil().matrix().cast<idx_t>() + vec3i_t::Ones();

  resize(offset, sizes, voxel_size);
}

void VoxelMap::resize(const vec3_t& offset, const vec3i_t& sizes, const coord_t voxel_size)
{
  m_voxel_vec = vec3_t(voxel_size, voxel_size, voxel_size);
  m_voxel_halfvec = m_voxel_vec/coord_t(2);
  m_voxel_size = voxel_size;
  m_voxel_size_inv = coord_t(1) / m_voxel_size;

  m_offset = offset;
  m_offset_x = offset.x();
  m_offset_y = offset.y();
  m_offset_z = offset.z();

  m_size_x = sizes.x();
  m_size_y = sizes.y();
  m_size_z = sizes.z();

  /* assertions (size checks) //{ */

  assert(m_size_x >= 0);
  assert(m_size_y >= 0);
  assert(m_size_z >= 0);
  assert(m_voxel_size_inv >= 0);

  //}

  const idx_t size_tot = idx_t(m_size_x) * idx_t(m_size_y) * idx_t(m_size_z);
  m_data.resize(size_tot);
}

void VoxelMap::resize(const coord_t center_x, const coord_t center_y, const coord_t center_z, const coord_t dimension_x, const coord_t dimension_y,
                      const coord_t dimension_z, const coord_t voxel_size)
{
  resize(vec3_t(center_x, center_y, center_z), vec3_t(dimension_x, dimension_y, dimension_z), voxel_size);
}

//}

/* resizeAs() method //{ */
void VoxelMap::resizeAs(const VoxelMap& size_as)
{
  resize(vec3_t(size_as.m_offset_x, size_as.m_offset_y, size_as.m_offset_z), vec3i_t(size_as.m_size_x, size_as.m_size_y, size_as.m_size_z),
             size_as.m_voxel_size);
}
//}

/* at() method //{ */
VoxelMap::data_t& VoxelMap::at(const coord_t x, const coord_t y, const coord_t z)
{
  /* assert(inLimits(x, y, z)); */
#ifndef NDEBUG
  if (!inLimits(x, y, z))
  {
    const auto [cx, cy, cz] = coordToIdx(x, y, z);
    std::cerr << "[" << x << ", " << y << ", " << z << "] is out of bounds (would have indices [" << cx << ", " << cy << ", " << cz << "], max indices are ["
              << m_size_x << ", " << m_size_y << ", " << m_size_z << "])" << std::endl;
    return m_data.at(0);
  }
#endif
  const auto [idx_x, idx_y, idx_z] = coordToIdx(x, y, z);

  const idx_t idx = idx_x + idx_y * m_size_x + idx_z * m_size_x * m_size_y;
  return m_data.at(idx);
}

VoxelMap::data_t VoxelMap::at(const coord_t x, const coord_t y, const coord_t z) const
{
  /* assert(inLimits(x, y, z)); */
#ifndef NDEBUG
  if (!inLimits(x, y, z))
  {
    const auto [cx, cy, cz] = coordToIdx(x, y, z);
    std::cerr << "[" << x << ", " << y << ", " << z << "] is out of bounds (would have indices [" << cx << ", " << cy << ", " << cz << "], max indices are ["
              << m_size_x << ", " << m_size_y << ", " << m_size_z << "])" << std::endl;
    return m_data.at(0);
  }
#endif
  const auto [idx_x, idx_y, idx_z] = coordToIdx(x, y, z);

  const idx_t idx = idx_x + idx_y * m_size_x + idx_z * m_size_x * m_size_y;
  return m_data.at(idx);
}
//}

/* atIdx() method //{ */
VoxelMap::data_t& VoxelMap::atIdx(const int idx_x, const int idx_y, const int idx_z)
{
  /* assert(inLimits(x, y, z)); */
#ifndef NDEBUG
  if (!inLimitsIdx(idx_x, idx_y, idx_z))
  {
    std::cerr << "[" << idx_x << ", " << idx_y << ", " << idx_z << "] index is out of bounds (max indices are [" << m_size_x << ", " << m_size_y << ", "
              << m_size_z << "])" << std::endl;
    return m_data.at(0);
  }
#endif
  const idx_t idx = idx_x + idx_y * m_size_x + idx_z * m_size_x * m_size_y;
  return m_data.at(idx);
}

VoxelMap::data_t VoxelMap::atIdx(const int idx_x, const int idx_y, const int idx_z) const
{
  /* assert(inLimits(x, y, z)); */
#ifndef NDEBUG
  if (!inLimitsIdx(idx_x, idx_y, idx_z))
  {
    std::cerr << "[" << idx_x << ", " << idx_y << ", " << idx_z << "] index is out of bounds (max indices are [" << m_size_x << ", " << m_size_y << ", "
              << m_size_z << "])" << std::endl;
    return m_data.at(0);
  }
#endif
  const idx_t idx = idx_x + idx_y * m_size_x + idx_z * m_size_x * m_size_y;
  return m_data.at(idx);
}

VoxelMap::data_t& VoxelMap::at(const vec3i_t& idx)
{
  return atIdx(idx.x(), idx.y(), idx.z());
}

VoxelMap::data_t VoxelMap::at(const vec3i_t& idx) const
{
  return atIdx(idx.x(), idx.y(), idx.z());
}

VoxelMap::data_t& VoxelMap::at(const idx3_t& idx)
{
  return atIdx(std::get<0>(idx), std::get<1>(idx), std::get<2>(idx));
}

VoxelMap::data_t VoxelMap::at(const idx3_t& idx) const
{
  return atIdx(std::get<0>(idx), std::get<1>(idx), std::get<2>(idx));
}
//}

/* voxelsAsPC() method //{ */
VoxelMap::pc_t::Ptr VoxelMap::voxelsAsPC(const data_t threshold, const bool greater_than, const pcl::PCLHeader& header)
{
  pc_t::Ptr cloud = boost::make_shared<pc_t>();
  cloud->reserve(m_size_x * m_size_y * m_size_z / 10);
  for (idx_t x_it = 0; x_it < m_size_x; x_it++)
  {
    for (idx_t y_it = 0; y_it < m_size_y; y_it++)
    {
      for (idx_t z_it = 0; z_it < m_size_z; z_it++)
      {
        const data_t mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        if ((mapval > threshold) == greater_than)
        {
          const auto [x, y, z] = idxToCoord(x_it, y_it, z_it);
          pt_t pt;
          pt.x = x;
          pt.y = y;
          pt.z = z;
          pt.intensity = mapval;
          cloud->push_back(pt);
        }
      }
    }
  }
  cloud->header = header;
  return cloud;
}
//}

/* voxelsAsVoxelPC() method //{ */
VoxelMap::pc_t::Ptr VoxelMap::voxelsAsVoxelPC(const data_t threshold, const bool greater_than, const pcl::PCLHeader& header)
{
  pc_t::Ptr cloud = boost::make_shared<pc_t>();
  cloud->reserve(m_size_x * m_size_y * m_size_z / 10);
  for (idx_t x_it = 0; x_it < m_size_x; x_it++)
  {
    for (idx_t y_it = 0; y_it < m_size_y; y_it++)
    {
      for (idx_t z_it = 0; z_it < m_size_z; z_it++)
      {
        const data_t mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        if ((mapval > threshold) == greater_than)
        {
          pt_t pt;
          pt.x = x_it;
          pt.y = y_it;
          pt.z = z_it;
          pt.intensity = mapval;
          cloud->push_back(pt);
        }
      }
    }
  }
  cloud->header = header;
  return cloud;
}
//}

/* nVoxelsOver() method //{ */
uint64_t VoxelMap::nVoxelsOver(const data_t threshold)
{
  uint64_t ret = 0;
  for (const auto val : m_data)
    ret += val > threshold;
  return ret;
}
//}

/* forEachRay() method //{ */

// implementation of:
// "A Fast Voxel Traversal Algorithm for Ray Tracing", John Amanatides, Andrew Woo
void VoxelMap::forEachRay(const vec3_t& start_pt, const vec3_t& dir, const coord_t length, const std::function<void(coord_t, const idx_t, const idx_t, const idx_t)> f)
{
  /* assert(dir.norm() == coord_t(1)); */
  const vec3_t absdir = dir.cwiseAbs();
  const vec3i_t step = dir.cwiseSign().cast<idx_t>(); // in what direction to step by cells
  const vec3_t tdelta = absdir.cwiseInverse()*m_voxel_size; // how far along the ray do we need to travel in units of *t* to move by m_voxel_size in the respective dimension
  vec3i_t cur_voxel = coordToIdx(start_pt); // indices of the voxel, corresponding to the starting point
  const vec3_t ctr_offset = idxToCoord(cur_voxel) - start_pt ; // offset of the starting point from the center of the corresponding voxel
  vec3_t tmax = ((m_voxel_halfvec.array() + step.array().cast<coord_t>()*ctr_offset.array())/absdir.array()).matrix(); // stores the value of *t* at which the next voxel boundary will be crossed
  assert((tmax.array() >= coord_t(0)).all());
/* nodelet: /home/matous/workspace/src/vofod/src/voxel_map.cpp:142: void VoxelMap::forEachRay(const vec3_t&, const vec3_t&, VoxelMap::coord_t, std::function<void(float, int, int, int)>): Assertion `(tmax.array() >= coord_t(0)).all()' failed. */ // <-- hopefully fixed now
  const vec3_t last_voxel(
      step.x() > idx_t(0) ? m_size_x-1 : 0,
      step.y() > idx_t(0) ? m_size_y-1 : 0,
      step.z() > idx_t(0) ? m_size_z-1 : 0
      );

  coord_t prev_dist = coord_t(0);
  while (prev_dist < length)
  {
    int i;
    const coord_t dist = tmax.minCoeff(&i);
    // call the requested function with the current state of the ray
    const coord_t ddist = std::min(dist, length) - prev_dist;
    f(ddist, cur_voxel.x(), cur_voxel.y(), cur_voxel.z());
    prev_dist = dist;

    // check if should end
    if (cur_voxel[i] == last_voxel[i])
      break;
    // update the state variables
    cur_voxel[i] += step[i];
    tmax[i] += tdelta[i];
  }
}

//}

/* clear() method //{ */
void VoxelMap::clear()
{
  setTo(0);
}
//}

/* setTo() method //{ */
void VoxelMap::setTo(const data_t value)
{
  std::fill(m_data.begin(), m_data.end(), value);
}
//}

/* copyDataIdx() method //{ */
void VoxelMap::copyDataIdx(const VoxelMap& from)
{
  m_data.assign(std::begin(from.m_data), std::end(from.m_data));
}
//}

/* inLimits() method //{ */
bool VoxelMap::inLimits(const coord_t x, const coord_t y, const coord_t z) const
{
  const auto [idx_x, idx_y, idx_z] = coordToIdx(x, y, z);
  return inLimitsIdx(idx_x, idx_y, idx_z);
}
//}

/* inLimitsIdx() method //{ */
bool VoxelMap::inLimitsIdx(const int idx_x, const int idx_y, const int idx_z) const
{
  return idx_x >= 0 && idx_x < m_size_x && idx_y >= 0 && idx_y < m_size_y && idx_z >= 0 && idx_z < m_size_z;
}
//}

/* inLimitsIdx() method //{ */
bool VoxelMap::inLimitsIdx(const vec3i_t& inds) const
{
  return inLimitsIdx(inds.x(), inds.y(), inds.z());
}
//}

/* manhattanDist() method //{ */
VoxelMap::idx_t VoxelMap::manhattanDist(const vec3i_t& inds1, const vec3i_t& inds2) const
{
  return (inds1 - inds2).cwiseAbs().sum();
}
//}

/* manhattanDist() method //{ */
VoxelMap::idx_t VoxelMap::manhattanDist(const idx3_t& inds1, const idx3_t& inds2) const
{
  return std::abs(std::get<0>(inds1) - std::get<0>(inds2))
       + std::abs(std::get<1>(inds1) - std::get<1>(inds2))
       + std::abs(std::get<2>(inds1) - std::get<2>(inds2));
}
//}

/* clearVisualizationThresholds() method //{ */
void VoxelMap::clearVisualizationThresholds()
{
  m_thresholds.clear();
}
//}

/* addVisualizationThreshold() method //{ */
void VoxelMap::addVisualizationThreshold(const data_t th, const std_msgs::ColorRGBA& th_color)
{
  m_thresholds.emplace_back(th, th_color);
  std::sort(std::begin(m_thresholds), std::end(m_thresholds), [](const auto& v1, const auto& v2){return v1.first < v2.first;});
}
//}

/* dimensions() method //{ */
// returns the dimensions (metric size) of the map
VoxelMap::vec3_t VoxelMap::dimensions() const
{
  return {m_voxel_size*m_size_x, m_voxel_size*m_size_y, m_voxel_size*m_size_z};
}
//}

/* origin() method //{ */
// returns the origin point of the map (also offset, the corner of the voxel at index [0,0,0])
VoxelMap::vec3_t VoxelMap::origin() const
{
  return m_offset;
}
//}

/* sizesIdx() method //{ */
// returns the sizes (max. indices) in each dimension
std::tuple<VoxelMap::idx_t, VoxelMap::idx_t, VoxelMap::idx_t> VoxelMap::sizesIdx() const
{
  return {m_size_x, m_size_y, m_size_z};
}
//}

// returns the sizes (max. indices) in each dimension
VoxelMap::vec3i_t VoxelMap::sizes() const
{
  return vec3i_t(m_size_x, m_size_y, m_size_z);
}

size_t VoxelMap::size() const
{
  return m_data.size();
}

bool VoxelMap::hasCloseTo(const coord_t x, const coord_t y, const coord_t z, const coord_t max_dist, const data_t threshold) const
{
  const vec3i_t orig_inds = coordToIdx(vec3_t(x, y, z));
  assert(inLimitsIdx(orig_inds));
  const coord_t max_dist_idx = max_dist*m_voxel_size_inv;
  const idx_t max_voxel_dist = std::ceil(max_dist_idx);
  const vec3i_t voxel_dists(max_voxel_dist, max_voxel_dist, max_voxel_dist);
  // clamp the ranges to be searched to the voxelmap dimensions
  const vec3i_t begin_inds = (orig_inds - voxel_dists).array().max(0);
  const vec3i_t end_inds = (orig_inds + voxel_dists).array().min(vec3i_t(m_size_x, m_size_y, m_size_z).array());

  for (idx_t x_it = begin_inds.x(); x_it < end_inds.x(); x_it++)
  {
    for (idx_t y_it = begin_inds.y(); y_it < end_inds.y(); y_it++)
    {
      for (idx_t z_it = begin_inds.z(); z_it < end_inds.z(); z_it++)
      {
        if (atIdx(x_it, y_it, z_it) > threshold && (vec3i_t(x_it, y_it, z_it) - orig_inds).norm() <= max_dist_idx)
          return true;
      }
    }
  }

  return false;
}

std::tuple<bool, std::vector<VoxelMap::idx3_t>> VoxelMap::exploreToGround(const coord_t x, const coord_t y, const coord_t z, const data_t unknown_threshold, const data_t ground_threshold, const coord_t max_voxel_dist) const
{
  // TODO: fix when the whole surrounding given by `max_voxel_dist` is explored and no path is found
  static const std::tuple<bool, std::vector<VoxelMap::idx3_t>> connected_ret = {true, {}};
  const auto orig_inds = coordToIdx(x, y, z);
  const auto [x_idx, y_idx, z_idx] = orig_inds;
  if (x_idx <= 0 || y_idx <= 0 || z_idx <= 0)
    return connected_ret;
  if (x_idx >= m_size_x - 1 || y_idx >= m_size_y - 1 || z_idx >= m_size_z - 1)
    return connected_ret;

  std::unordered_set<idx3_t, hash_tuple::hash<std::tuple<int, int, int>>> explored;
  std::vector<idx3_t> explored_unknown;
  std::vector<idx3_t> to_explore;
  to_explore.push_back({x_idx, y_idx, z_idx});

  while (!to_explore.empty())
  {
    const auto cur_idx = to_explore.back();
    to_explore.pop_back(); // DFS
    const auto cur_val = at(cur_idx);

    if (cur_val > ground_threshold)
      return connected_ret;
    if (cur_val > unknown_threshold)
    {
      explored_unknown.push_back(cur_idx);
      // check if we're at the edge of the search area
      if (manhattanDist(orig_inds, cur_idx) == max_voxel_dist-1)
        return connected_ret; // if so, consider this cluster to be connected to ground

      idx3_t to_add; // just a helper variable

      /* expand in the positive directions of the coordinates //{ */
      
      if (std::get<0>(cur_idx) < m_size_x-1)
      {
        to_add = {std::get<0>(cur_idx)+1, std::get<1>(cur_idx), std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<1>(cur_idx) < m_size_y-1)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx)+1, std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<2>(cur_idx) < m_size_z-1)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx), std::get<2>(cur_idx)+1};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      
      //}

      /* expand in the negative directions of the coordinates //{ */
      
      if (std::get<0>(cur_idx) > 0)
      {
        to_add = {std::get<0>(cur_idx)-1, std::get<1>(cur_idx), std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<1>(cur_idx) > 0)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx)-1, std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<2>(cur_idx) > 0)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx), std::get<2>(cur_idx)-1};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      
      //}
      
      // TODO: explore the whole 26-surrounding
    }

    explored.insert(std::move(cur_idx));
  }

  return {false, explored_unknown};
}

// returns true if no voxel around (including) this one is above the threshold
bool VoxelMap::isFloating(const coord_t x, const coord_t y, const coord_t z, const data_t threshold) const
{
  const auto [x_idx, y_idx, z_idx] = coordToIdx(x, y, z);
  return isFloatingIdx(x_idx, y_idx, z_idx, threshold);
}

bool VoxelMap::isFloatingIdx(const idx_t x_idx, const idx_t y_idx, const idx_t z_idx, const data_t threshold) const
{
  if (x_idx <= 0 || y_idx <= 0 || z_idx <= 0)
    return false;
  if (x_idx >= m_size_x - 1 || y_idx >= m_size_y - 1 || z_idx >= m_size_z - 1)
    return false;

  for (idx_t x_it = x_idx - 1; x_it <= x_idx + 1; x_it++)
  {
    for (idx_t y_it = y_idx - 1; y_it <= y_idx + 1; y_it++)
    {
      for (idx_t z_it = z_idx - 1; z_it <= z_idx + 1; z_it++)
      {
        if (m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y) > threshold)
          return false;
      }
    }
  }
  return true;
}

void VoxelMap::forEachIdx(const std::function<void(data_t&, const idx_t, const idx_t, const idx_t)> f, const idx_t offset)
{
  const auto max_x = m_size_x - offset;
  const auto max_y = m_size_y - offset;
  const auto max_z = m_size_z - offset;
  for (idx_t x_it = offset; x_it < max_x; x_it++)
  {
    for (idx_t y_it = offset; y_it < max_y; y_it++)
    {
      for (idx_t z_it = offset; z_it < max_z; z_it++)
      {
        data_t& mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        f(mapval, x_it, y_it, z_it);
      }
    }
  }
}

void VoxelMap::forEach(const std::function<void(data_t&, const coord_t, const coord_t, const coord_t)> f, const idx_t offset)
{
  forEachIdx(
      [f, this](VoxelMap::data_t& val, const int x_it, const int y_it, const int z_it) {
        const auto [x, y, z] = idxToCoord(x_it, y_it, z_it);
        f(val, x, y, z);
      },
      offset);
}

/* getSubmapCopy() method //{ */
VoxelMap VoxelMap::getSubmapCopy(const vec3_t& min_pt, const vec3_t& max_pt, const int inflate)
{
  assert((min_pt.array() <= max_pt.array()).all());
  vec3i_t min_inds = coordToIdx(min_pt);
  vec3i_t max_inds = coordToIdx(max_pt);
  // clamp min indices
  min_inds.x() = std::clamp(min_inds.x() - inflate, 0, m_size_x - 1);
  min_inds.y() = std::clamp(min_inds.y() - inflate, 0, m_size_y - 1);
  min_inds.z() = std::clamp(min_inds.z() - inflate, 0, m_size_z - 1);
  // clamp max indices
  max_inds.x() = std::clamp(max_inds.x() + inflate, 0, m_size_x - 1);
  max_inds.y() = std::clamp(max_inds.y() + inflate, 0, m_size_y - 1);
  max_inds.z() = std::clamp(max_inds.z() + inflate, 0, m_size_z - 1);
  assert(inLimitsIdx(min_inds));
  assert(inLimitsIdx(max_inds));

  const vec3_t submap_offset = idxToCoord(min_inds) - vec3_t(m_voxel_size, m_voxel_size, m_voxel_size) / coord_t(2);
  const vec3i_t submap_size = max_inds - min_inds + vec3i_t::Ones();

  VoxelMap ret;
  ret.resize(submap_offset, submap_size, m_voxel_size);

  for (idx_t x_it = 0; x_it < submap_size.x(); x_it++)
  {
    for (idx_t y_it = 0; y_it < submap_size.y(); y_it++)
    {
      for (idx_t z_it = 0; z_it < submap_size.z(); z_it++)
      {
        const int idx_x = x_it + min_inds.x();
        const int idx_y = y_it + min_inds.y();
        const int idx_z = z_it + min_inds.z();
        const data_t mapval = m_data.at(idx_x + idx_y * m_size_x + idx_z * m_size_x * m_size_y);
        ret.atIdx(x_it, y_it, z_it) = mapval;
      }
    }
  }
  return ret;
}
//}

/* VoxelMap::data_t VoxelMap::min() */
/* { */
/*   return *std::min_element(std::begin(m_data), std::end(m_data)); */
/* } */

std::tuple<VoxelMap::idx_t, VoxelMap::idx_t, VoxelMap::idx_t> VoxelMap::coordToIdx(const coord_t x, const coord_t y, const coord_t z) const
{
  // optimization potential here: precompute m_center_X*m_voxel_size_inv and use x*m_voxel_size_inv + m_center_X to enable MADD
  const idx_t idx_x = std::floor((x - m_offset_x) * m_voxel_size_inv);
  const idx_t idx_y = std::floor((y - m_offset_y) * m_voxel_size_inv);
  const idx_t idx_z = std::floor((z - m_offset_z) * m_voxel_size_inv);
  return {idx_x, idx_y, idx_z};
}

VoxelMap::vec3i_t VoxelMap::coordToIdx(const vec3_t& coords) const
{
  const auto [idx_x, idx_y, idx_z] = coordToIdx(coords.x(), coords.y(), coords.z());
  return vec3i_t(idx_x, idx_y, idx_z);
}

std::tuple<VoxelMap::coord_t, VoxelMap::coord_t, VoxelMap::coord_t> VoxelMap::idxToCoord(const idx_t x_idx, const idx_t y_idx, const idx_t z_idx) const
{
  const coord_t x = (x_idx + coord_t(0.5)) * m_voxel_size + m_offset_x;
  const coord_t y = (y_idx + coord_t(0.5)) * m_voxel_size + m_offset_y;
  const coord_t z = (z_idx + coord_t(0.5)) * m_voxel_size + m_offset_z;
  return {x, y, z};
}

VoxelMap::vec3_t VoxelMap::idxToCoord(const vec3i_t& inds) const
{
  const auto [x, y, z] = idxToCoord(inds.x(), inds.y(), inds.z());
  return vec3_t(x, y, z);
}

/* visualization() method //{ */
visualization_msgs::Marker VoxelMap::visualization(const std_msgs::Header& header) const
{
  visualization_msgs::Marker ret;
  ret.header = header;
  ret.points.reserve(m_data.size() / 1000);
  ret.pose.position.x = m_offset_x + m_voxel_size / coord_t(2);
  ret.pose.position.y = m_offset_y + m_voxel_size / coord_t(2);
  ret.pose.position.z = m_offset_z + m_voxel_size / coord_t(2);
  ret.pose.orientation.w = 1.0;
  ret.scale.x = ret.scale.y = ret.scale.z = m_voxel_size;
  ret.color.a = 1.0;
  ret.type = visualization_msgs::Marker::CUBE_LIST;

  for (idx_t x_it = 0; x_it < m_size_x; x_it++)
  {
    for (idx_t y_it = 0; y_it < m_size_y; y_it++)
    {
      for (idx_t z_it = 0; z_it < m_size_z; z_it++)
      {
        const data_t mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        // find the color of this voxel
        bool found = false;
        std_msgs::ColorRGBA color;
        for (const auto& [th, clr] : m_thresholds)
        {
          if (mapval > th)
          {
            color = clr;
            found = true;
          }
        }
        // empty voxels
        if (!found)
          continue;

        geometry_msgs::Point pt;
        pt.x = x_it * m_voxel_size;
        pt.y = y_it * m_voxel_size;
        pt.z = z_it * m_voxel_size;
        ret.points.push_back(pt);
        ret.colors.push_back(color);
      }
    }
  }

  return ret;
}
//}

/* borderVisualization() method //{ */
visualization_msgs::Marker VoxelMap::borderVisualization(const std_msgs::Header& header) const
{
  visualization_msgs::Marker ret;
  ret.header = header;
  ret.points.reserve(24);
  ret.pose.position.x = m_offset_x;
  ret.pose.position.y = m_offset_y;
  ret.pose.position.z = m_offset_z;
  ret.pose.orientation.w = 1.0;
  ret.scale.x = 0.05;
  ret.color.r = 1.0;
  ret.color.g = 1.0;
  ret.color.b = 1.0;
  ret.color.a = 1.0;
  ret.type = visualization_msgs::Marker::LINE_LIST;

  const coord_t dim_x = m_size_x * m_voxel_size;
  const coord_t dim_y = m_size_y * m_voxel_size;
  const coord_t dim_z = m_size_z * m_voxel_size;

  geometry_msgs::Point pt;

  /* bottom square //{ */

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = 0.0;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = 0.0;
  ret.points.push_back(pt);

  //}

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  /* top square //{ */

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  //}

  // remaining connections
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = 0.0;
  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  return ret;
}
//}
