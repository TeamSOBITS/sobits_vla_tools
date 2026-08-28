cmake_minimum_required(VERSION 3.10.2)
project(sobits_vla_rosbag_collection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sobits_interfaces REQUIRED)

find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sobits_vla_common REQUIRED)


# Find libraries
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/robot_descriptor_loader.cpp
  src/recording_monitor.cpp
  src/bag_metadata_manager.cpp
  src/topic_builder.cpp
  src/rosbag_collection.cpp
)

# Link libraries
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobits_vla::RosbagCollection"
  EXECUTABLE rosbag_collection_node
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "rosbag2_cpp"
  "rosbag2_storage"
  "sensor_msgs"
  "rcl_interfaces"
  "sobits_interfaces"
  "yaml_cpp_vendor"
  "rosbag2_transport"
  "sobits_vla_common"
)

# yaml-cpp is found above and its headers are included directly; link it
# explicitly rather than relying on yaml_cpp_vendor to propagate it.
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Headers are internal to this package: no downstream package includes them,
# so they are deliberately not installed or exported.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
