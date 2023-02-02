cmake_minimum_required(VERSION 3.18)
project(mama_server)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "Use C++20 by default")

add_subdirectory(.. ${CMAKE_CURRENT_BINARY_DIR}/mama EXCLUDE_FROM_ALL)

# we use system's SSL to avoid issues on M1 macs we currently have with gRPC's bundled SSL
set(gRPC_SSL_PROVIDER "package"  CACHE STRING "")
# we use system's protobuf, because the rest of the project uses it too 
set(gRPC_PROTOBUF_PROVIDER "package"  CACHE STRING "")
set(gRPC_PROTOBUF_PACKAGE_TYPE "MODULE"  CACHE STRING "")
find_package(Protobuf REQUIRED)

add_subdirectory(../thirdparty/grpc ${CMAKE_CURRENT_BINARY_DIR}/grpc EXCLUDE_FROM_ALL)
set(_PROTOBUF_LIBPROTOBUF libprotobuf)
set(_REFLECTION grpc++_reflection)
set(_GRPC_GRPCPP grpc++)
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)

get_filename_component(mama_proto "mama.proto" ABSOLUTE)
get_filename_component(mama_proto_path "${mama_proto}" PATH)

add_custom_command(
      OUTPUT 
        ${CMAKE_CURRENT_BINARY_DIR}/mama.pb.cc
        ${CMAKE_CURRENT_BINARY_DIR}/mama.pb.h
        ${CMAKE_CURRENT_BINARY_DIR}/mama.grpc.pb.cc
        ${CMAKE_CURRENT_BINARY_DIR}/mama.grpc.pb.h
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${mama_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${mama_proto}"
      DEPENDS "${mama_proto}")

add_library(mama_grpc_proto
  ${CMAKE_CURRENT_BINARY_DIR}/mama.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/mama.pb.h
  ${CMAKE_CURRENT_BINARY_DIR}/mama.grpc.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/mama.grpc.pb.h)
target_link_libraries(mama_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${PROTOBUF_LIBRARIES})

add_executable(mama_server "mama_server.cc")
target_link_libraries(mama_server
    PRIVATE
    mama
    mama_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${PROTOBUF_LIBRARIES})
# TODO: mama should export its public include directories
target_include_directories(mama_server
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../src
    SYSTEM PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/spdlog/include
      ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/geometry.hpp/include
      ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/cheap-ruler-cpp/include
)
