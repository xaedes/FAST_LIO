cmake_minimum_required(VERSION 3.8)

find_package(OpenMP QUIET)


find_package(Eigen3 REQUIRED)
# find_package(PCL 1.8 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)
message("PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS}")
find_package(Boost REQUIRED)
