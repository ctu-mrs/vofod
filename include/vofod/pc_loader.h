#ifndef PC_LOADER_H
#define PC_LOADER_H

#include <pcl/io/pcd_io.h>

using pt_t = pcl::PointXYZ;
using pc_t = pcl::PointCloud<pt_t>;

pc_t::Ptr load_cloud(const std::string& filename);

#endif // PC_LOADER_H
