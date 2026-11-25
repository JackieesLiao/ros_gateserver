cmake_minimum_required(VERSION 3.16)
project(ros_gateserver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gateserver_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED CONFIG)
find_program(GRPC_PYTHON_PLUGIN grpc_python_plugin)
if(NOT GRPC_PYTHON_PLUGIN)
    message(FATAL_ERROR "grpc_python_plugin is required. Install protobuf-compiler-grpc.")
endif()
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
# TF2 and geometry messages dependencies
find_package(geometry_msgs REQUIRED)  # 带坐标系、时间戳的点和变换消息
find_package(tf2 REQUIRED)            # 坐标变换的核心能力
find_package(tf2_ros REQUIRED)        # TF 广播器、监听器和缓存
find_package(tf2_geometry_msgs REQUIRED)  # 让 tf2 支持转换 PointStamped 等消息

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS proto/gateway_status.proto)
protobuf_generate_python(PROTO_PYTHON proto/gateway_status.proto)
set(STATUS_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(STATUS_PROTO "${STATUS_PROTO_DIR}/gateway_status.proto")
set(GRPC_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/gateway_status.grpc.pb.cc")
set(GRPC_HEADER "${CMAKE_CURRENT_BINARY_DIR}/gateway_status.grpc.pb.h")
set(GRPC_PYTHON "${CMAKE_CURRENT_BINARY_DIR}/ros_gateserver_protocol/gateway_status_pb2_grpc.py")

add_custom_command(
    OUTPUT "${GRPC_SOURCE}" "${GRPC_HEADER}"
    COMMAND protobuf::protoc
        "--proto_path=${STATUS_PROTO_DIR}"
        "--grpc_out=${CMAKE_CURRENT_BINARY_DIR}"
        "--plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
        "${STATUS_PROTO}"
    DEPENDS "${STATUS_PROTO}" protobuf::protoc gRPC::grpc_cpp_plugin
    VERBATIM
)

# The virtual path makes the generated stub import the installed Python package.
add_custom_command(
    OUTPUT "${GRPC_PYTHON}"
    COMMAND protobuf::protoc
        "--proto_path=ros_gateserver_protocol=${STATUS_PROTO_DIR}"
        "--grpc_out=${CMAKE_CURRENT_BINARY_DIR}"
        "--plugin=protoc-gen-grpc=${GRPC_PYTHON_PLUGIN}"
        "${STATUS_PROTO}"
    DEPENDS "${STATUS_PROTO}" protobuf::protoc "${GRPC_PYTHON_PLUGIN}"
    VERBATIM
)

add_library(gateway_protocol STATIC src/status_codec.cpp ${PROTO_SOURCES} ${PROTO_HEADERS})
target_include_directories(gateway_protocol PUBLIC include ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(gateway_protocol PUBLIC protobuf::libprotobuf nlohmann_json::nlohmann_json)
add_custom_target(gateway_protocol_python ALL DEPENDS ${PROTO_PYTHON} "${GRPC_PYTHON}")

# Python 客户端和测试复用同一份 protoc 生成文件，避免手写二进制解析。
ament_python_install_package(ros_gateserver_protocol PACKAGE_DIR python/ros_gateserver_protocol)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES ${PROTO_PYTHON} "${GRPC_PYTHON}"
    DESTINATION ${PYTHON_INSTALL_DIR}/ros_gateserver_protocol
)

add_executable(gateway
    src/gateway.cpp src/grpc_status_server.cpp src/image_quality.cpp "${GRPC_SOURCE}"
)
target_include_directories(gateway PRIVATE include ${OpenCV_INCLUDE_DIRS})
# 添加 TF2 和几何消息的依赖
ament_target_dependencies(
    gateway
    rclcpp
    sensor_msgs
    nav_msgs
    std_msgs
    std_srvs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
)
target_link_libraries(gateway
    gateway_protocol gRPC::grpc++ Boost::system nlohmann_json::nlohmann_json Threads::Threads
    ${OpenCV_LIBRARIES}
)

add_executable(sensor_check_server src/sensor_check_server.cpp)
ament_target_dependencies(sensor_check_server
    rclcpp rclcpp_action ros_gateserver_interfaces std_msgs
)
target_link_libraries(sensor_check_server nlohmann_json::nlohmann_json)

add_executable(mock_sensors src/mock_sensors.cpp)
ament_target_dependencies(mock_sensors rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)

foreach(target gateway mock_sensors sensor_check_server gateway_protocol)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS gateway mock_sensors sensor_check_server DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/record_bag.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME record_bag
)
install(PROGRAMS scripts/protobuf_client.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME protobuf_client
)
install(PROGRAMS scripts/grpc_client.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME grpc_client
)
install(DIRECTORY launch config proto DESTINATION share/${PROJECT_NAME})

ament_package()
