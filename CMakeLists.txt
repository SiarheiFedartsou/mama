cmake_minimum_required(VERSION 3.18)
project(mama)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "Use C++20 by default")

find_package(ZLIB REQUIRED)
add_subdirectory(thirdparty/abseil-cpp SYSTEM)
add_subdirectory(thirdparty/s2geometry SYSTEM)

# protobuf
find_package(Protobuf REQUIRED)
# for generated headers
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/protos/tile.proto src/protos/state.proto)

add_library(generated_proto 
    STATIC
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)
target_link_libraries(generated_proto PRIVATE ${Protobuf_LIBRARIES}) 
target_include_directories(generated_proto SYSTEM PUBLIC ${Protobuf_INCLUDE_DIRS})

add_executable(
    tilegen 
    src/tilegen/tilegen.cc 
    src/tilegen/osm_data_collector.hpp 
    src/tilegen/osm_data_collector.cc  
    src/tilegen/options.hpp 
    src/tilegen/options.cc
)
target_link_libraries(tilegen PRIVATE s2 generated_proto ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES}) 
target_include_directories(tilegen 
    PRIVATE 
        src
    SYSTEM PRIVATE 
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/protozero/include 
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/libosmium/include  
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/geometry.hpp/include
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/cheap-ruler-cpp/include 
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/spdlog/include
)

add_library(mama src/mama.cc src/graph/graph.cc src/graph/coverer.cc)
target_link_libraries(mama PUBLIC s2 PRIVATE generated_proto ${EXPAT_LIBRARIES} ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES}) 
target_include_directories(
    mama 
    PRIVATE src 
    SYSTEM PRIVATE 
        ${CMAKE_CURRENT_BINARY_DIR}
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



include(${CMAKE_CURRENT_LIST_DIR}/cmake/warnings.cmake)
configure_warnings(tilegen)
configure_warnings(mama)
configure_warnings(tests)
