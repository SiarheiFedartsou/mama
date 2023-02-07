cmake_minimum_required(VERSION 3.18)
project(mama)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "Use C++20 by default")

find_package(ZLIB REQUIRED)
add_subdirectory(thirdparty/abseil-cpp)
add_subdirectory(thirdparty/s2geometry)

# protobuf
find_package(Protobuf REQUIRED)
# for generated headers
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/protos/tile.proto src/protos/state.proto)

add_executable(tilegen src/tilegen/tilegen.cc  ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(tilegen PRIVATE s2 ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} ${Protobuf_LIBRARIES}) 
target_include_directories(tilegen 
    PRIVATE 
        src
    SYSTEM PRIVATE 
        ${CMAKE_CURRENT_BINARY_DIR} 
        ${Protobuf_INCLUDE_DIRS} 
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/protozero/include 
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/libosmium/include  
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/geometry.hpp/include
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cheap-ruler-cpp/include 
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/spdlog/include
)

add_library(mama src/mama.cc src/graph/graph.cc ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(mama PUBLIC s2 PRIVATE ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} ${Protobuf_LIBRARIES}) 
target_include_directories(
    mama 
    PRIVATE src 
    SYSTEM PRIVATE 
        ${CMAKE_CURRENT_BINARY_DIR} 
        ${Protobuf_INCLUDE_DIRS} 
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/spdlog/include

        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/geometry.hpp/include
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cheap-ruler-cpp/include
)

add_subdirectory(thirdparty/Catch2)
add_executable(tests test/base/angle_tests.cc test/base/coordinate_tests.cc test/graph/graph_tests.cc)
target_link_libraries(tests PRIVATE mama Catch2::Catch2 Catch2::Catch2WithMain)
target_include_directories(tests PRIVATE 
    src 
    ${CMAKE_CURRENT_LIST_DIR}/thirdparty/geometry.hpp/include
    ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cheap-ruler-cpp/include)

install(TARGETS tilegen DESTINATION bin)
