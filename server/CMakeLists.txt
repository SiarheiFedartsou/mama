cmake_minimum_required(VERSION 3.18)
project(mama_server)

# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")


set(CMAKE_CXX_STANDARD 20 CACHE STRING "Use C++20 by default")

add_subdirectory(.. ${CMAKE_CURRENT_BINARY_DIR}/mama EXCLUDE_FROM_ALL)


find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

get_filename_component(hw_proto "mama.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/mama.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/mama.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/mama.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/mama.grpc.pb.h")

find_program(GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin)
find_program(PROTOC NAMES protoc)
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")
# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# hw_grpc_proto
add_library(hw_grpc_proto
  ${hw_proto_srcs}
  ${hw_proto_hdrs}
  ${hw_grpc_srcs}
  ${hw_grpc_hdrs})
target_link_libraries(hw_grpc_proto
  gRPC::grpc++
  ${Protobuf_LIBRARIES})

add_executable(mama_server "mama_server.cc")
target_link_libraries(mama_server
    PRIVATE
    mama
    hw_grpc_proto
    gRPC::grpc++
    ${Protobuf_LIBRARIES})
# TODO: mama should export its public include directories
target_include_directories(mama_server
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
