cmake_minimum_required(VERSION 2.8.3)
# to make CMake shut up about some fckin policies
cmake_policy(SET CMP0048 NEW)
project(vofod VERSION 1.0)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

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

###########
## Build ##
###########

cs_add_library(VoFOD src/vofod_nodelet.cpp src/voxel_map.cpp src/voxel_grid_weighted.cpp src/voxel_grid_counted.cpp src/pc_loader.cpp)

cs_add_library(MaskCreator src/mask_creator.cpp)

cs_export()
