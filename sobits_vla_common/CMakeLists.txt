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
  scripts/world_reset_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependency data
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs sobits_interfaces rclcpp_components)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # lerobot seam smoke-test suite — the pass/fail gate for lerobot upgrades.
  # lerobot lives in the repo's pixi environments (not system python), so
  # the test runs on the first materialized env interpreter that ships both
  # pytest and lerobot. Standalone equivalent:
  #   pixi run -e gpu python -m pytest sobits_vla_common/test/ -v
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    set(_pixi_envs_dir "${CMAKE_CURRENT_SOURCE_DIR}/../.pixi/envs")
    set(_seam_probe_py
      "import importlib.util as u, sys; sys.exit(int(not (u.find_spec('pytest') and u.find_spec('lerobot'))))")
    set(_seam_env_candidates gpu cpu)
    set(_seam_python "")
    foreach(_env ${_seam_env_candidates})
      set(_cand "${_pixi_envs_dir}/${_env}/bin/python")
      if(EXISTS "${_cand}" AND _seam_python STREQUAL "")
        execute_process(
          COMMAND "${_cand}" -c "${_seam_probe_py}"
          RESULT_VARIABLE _seam_probe
          OUTPUT_QUIET
          ERROR_QUIET
        )
        if(_seam_probe EQUAL 0)
          set(_seam_python "${_cand}")
        endif()
      endif()
    endforeach()
    if(_seam_python STREQUAL "")
      message(STATUS "lerobot seam suite: no pixi env with pytest+lerobot"
        " - test skipped (run 'pixi install' in sobits_vla_tools)")
    else()
      message(STATUS "lerobot seam suite: using ${_seam_python}")
      # PYTEST_DISABLE_PLUGIN_AUTOLOAD: the appended ROS PYTHONPATH exposes
      # ROS pytest plugins (launch_testing -> rclpy -> lark) that the pixi
      # interpreter cannot import; the suite needs no plugins.
      # LD_LIBRARY_PATH: colcon runs this interpreter without the pixi
      # activation, so prepend the env's libs as scripts/pixi_ros_bridge.sh does.
      get_filename_component(_seam_env_root "${_seam_python}" DIRECTORY)
      get_filename_component(_seam_env_root "${_seam_env_root}" DIRECTORY)
      ament_add_pytest_test(test_lerobot_seam test/test_lerobot_seam.py
        PYTHON_EXECUTABLE "${_seam_python}"
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
          PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
        PREPEND_ENV LD_LIBRARY_PATH=${_seam_env_root}/lib
        TIMEOUT 120
      )
    endif()

    # world_reset / gz_utils unit tests -- pure Python, only need pytest and
    # PyYAML (no lerobot, no rclpy). Reuse any pixi env that has them.
    set(_wr_probe_py
      "import importlib.util as u, sys; sys.exit(int(not (u.find_spec('pytest') and u.find_spec('yaml'))))")
    set(_wr_python "")
    if(NOT _seam_python STREQUAL "")
      set(_wr_python "${_seam_python}")
    else()
      foreach(_env ${_seam_env_candidates})
        set(_cand "${_pixi_envs_dir}/${_env}/bin/python")
        if(EXISTS "${_cand}" AND _wr_python STREQUAL "")
          execute_process(
            COMMAND "${_cand}" -c "${_wr_probe_py}"
            RESULT_VARIABLE _wr_probe
            OUTPUT_QUIET
            ERROR_QUIET
          )
          if(_wr_probe EQUAL 0)
            set(_wr_python "${_cand}")
          endif()
        endif()
      endforeach()
    endif()
    if(_wr_python STREQUAL "")
      message(STATUS "world_reset/gz_utils tests: no pixi env with pytest+yaml"
        " - test skipped (run 'pixi install' in sobits_vla_tools)")
    else()
      message(STATUS "world_reset/gz_utils tests: using ${_wr_python}")
      ament_add_pytest_test(test_world_reset test/test_world_reset.py
        PYTHON_EXECUTABLE "${_wr_python}"
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
          PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
        TIMEOUT 60
      )
      ament_add_pytest_test(test_gz_utils test/test_gz_utils.py
        PYTHON_EXECUTABLE "${_wr_python}"
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
          PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
        TIMEOUT 60
      )
    endif()
  endif()
endif()

ament_package()
