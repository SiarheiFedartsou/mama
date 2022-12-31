cmake_minimum_required(VERSION 3.18)
project(mama)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "Use C++20 by default")

find_package(EXPAT REQUIRED)
find_package(BZip2 REQUIRED)
find_package(ZLIB REQUIRED)
add_subdirectory(thirdparty/abseil-cpp)
add_subdirectory(thirdparty/s2geometry)
include_directories(SYSTEM thirdparty/protozero/include)
include_directories(SYSTEM thirdparty/libosmium/include)

# protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${Protobuf_INCLUDE_DIRS})
# for generated headers
include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/protos/tile.proto src/protos/state.proto)

add_executable(tilegen src/tilegen/tilegen.cc  ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(tilegen PRIVATE s2 ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} ${Protobuf_LIBRARIES}) 
target_include_directories(tilegen PRIVATE src)

add_library(mama src/mama.cc src/graph/graph.cc ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(mama PUBLIC s2 PRIVATE ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} ${Protobuf_LIBRARIES}) 
target_include_directories(mama PRIVATE src)

add_executable(cli src/cli.cc)
target_link_libraries(cli PRIVATE mama)
target_include_directories(cli PRIVATE src)

