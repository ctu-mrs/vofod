cmake_minimum_required(VERSION 2.8.3)
# to make CMake shut up about some fckin policies
cmake_policy(SET CMP0048 NEW)
project(vofod VERSION 666)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wpedantic)

add_compile_options(-DNDEBUG)
add_compile_options(-O3)

# add_compile_options("-DEIGEN_INITIALIZE_MATRICES_BY_NAN")
# add_compile_options("-DDISABLE_PARALLELIZATION")
# add_compile_options(-g)
# add_compile_options(-Og)
# add_definitions(-fsanitize=address,undefined,alignment,bounds)
# add_link_options(-fsanitize=address,undefined,alignment,bounds)
# add_definitions(-fsanitize=thread)
# add_link_options(-fsanitize=thread)

# remove warning info from PCL
set(PCL_FIND_QUIETLY INTERNAL)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  cv_bridge
  image_transport
  image_geometry
  message_runtime
  message_generation
  tf2
  tf2_ros
  tf2_eigen
  ouster_ros
  mrs_lib
  pcl_conversions
  pcl_ros
  pcl_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  sensor_msgs
  mavros_msgs
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)

find_package(pcl_ros REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)

add_message_files(DIRECTORY msgs FILES
  Detection.msg
  DetectionStamped.msg
  Detections.msg
  Status.msg
  ProfilingInfo.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/DetectionParams.cfg
  )

catkin_package(CATKIN_DEPENDS
  roscpp
  nodelet
  image_transport
  message_runtime
  dynamic_reconfigure
  mrs_lib
  ouster_ros
  pcl_ros
  pcl_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
  tf2_sensor_msgs
  visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

add_library(PCLDetector src/pcl_detect_nodelet.cpp src/voxel_map.cpp src/voxel_grid_weighted.cpp src/voxel_grid_counted.cpp src/pcl_loader.cpp)
add_dependencies(PCLDetector
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(PCLDetector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)
target_include_directories(PCLDetector SYSTEM PRIVATE ${pcl_ros_INCLUDE_DIRS})

add_library(MaskCreator src/mask_creator.cpp)
add_dependencies(MaskCreator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(MaskCreator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)
target_include_directories(MaskCreator SYSTEM PRIVATE ${pcl_ros_INCLUDE_DIRS})
