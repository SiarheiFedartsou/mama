cmake_minimum_required(VERSION 3.21)
project(mama)

set(CMAKE_CXX_STANDARD 20)

# add_subdirectory(abseil-cpp)
# add_subdirectory(s2geometry)
# include_directories(SYSTEM libosmium/include)

find_package(EXPAT REQUIRED)
find_package(BZip2 REQUIRED)
find_package(ZLIB REQUIRED)
# find_library(S2_LIBRARY NAMES s2 s2_geometry REQUIRED)
add_subdirectory(abseil-cpp)
# TODO: override set(CMAKE_CXX_STANDARD 20)
add_subdirectory(s2geometry)
include_directories(SYSTEM protozero/include)
include_directories(SYSTEM libosmium/include)

include_directories(SYSTEM /opt/homebrew/include)
add_executable(point_index point_index.cc)
target_link_libraries(point_index PUBLIC s2 ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES}) 


# 