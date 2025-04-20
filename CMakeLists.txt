cmake_minimum_required(VERSION 3.8)
project(fcuflex_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find additional system dependencies
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(nlohmann_json REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AFDStatus.msg"
  "srv/GetValue.srv"
  "srv/SetValue.srv"
  DEPENDENCIES std_msgs
)

# Generate the C++ interfaces for our custom messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Create a library for the client implementation
add_library(fcuflex_client
  src/fcuflex_client.cpp
)

target_include_directories(fcuflex_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fcuflex_client
  ${Boost_LIBRARIES}
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(fcuflex_client
  rclcpp
)

# Create the node executable
add_executable(fcuflex_node
  src/fcuflex_node.cpp
)

target_include_directories(fcuflex_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fcuflex_node
  fcuflex_client
  ${cpp_typesupport_target}
)

ament_target_dependencies(fcuflex_node
  rclcpp
  std_msgs
  std_srvs
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS fcuflex_client
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS fcuflex_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(fcuflex_client)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  rosidl_default_runtime
)

ament_package()