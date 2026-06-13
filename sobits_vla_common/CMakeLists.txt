cmake_minimum_required(VERSION 3.12)
project(sobits_vla_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(include)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# C++ Gamepad Client Library component
add_library(${PROJECT_NAME} SHARED
  src/gamepad_clt.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "sensor_msgs"
  "sobits_interfaces"
  "rclcpp_components"
)

# Register component plugin
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "sobits_vla::GamepadClient"
  EXECUTABLE gamepad_clt_node
)

# Install C++ library targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install C++ headers
install(DIRECTORY include/
  DESTINATION include
)

# Install robot descriptors and gamepad config files
install(DIRECTORY robots config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS
  scripts/new_robot
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependency data
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs sobits_interfaces rclcpp_components)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright checking
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
