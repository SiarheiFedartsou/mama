cmake_minimum_required(VERSION 3.18)
project(mama)

set(CMAKE_CXX_STANDARD 20)

find_package(EXPAT REQUIRED)
find_package(BZip2 REQUIRED)
find_package(ZLIB REQUIRED)
add_subdirectory(abseil-cpp)
# TODO: override set(CMAKE_CXX_STANDARD 20)
add_subdirectory(s2geometry)
include_directories(SYSTEM protozero/include)
include_directories(SYSTEM libosmium/include)

# protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${Protobuf_INCLUDE_DIRS})
# for generated headers
include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS tile.proto state.proto)

add_executable(tilegen tilegen.cc  ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(tilegen PUBLIC s2 ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} ${Protobuf_LIBRARIES}) 

add_executable(mama mama.cc  ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(mama PUBLIC s2 ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} ${Protobuf_LIBRARIES}) 
