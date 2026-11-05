cmake_minimum_required(VERSION 3.8)
project(luna_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)

set(PY_EXECUTABLES
  arena_gen
  kinematic_base
  oracle_localizer
  sim_clock
  mock_obstacle_detector
  mock_wheel_odometry
  mock_tag_detector
  mock_excavation
  mock_dump
  mock_imu
  # scorer
)

foreach(exe ${PY_EXECUTABLES})
  install(PROGRAMS scripts/${exe}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${exe})
endforeach()

install(DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test_name test_imu_model test_mock_imu_node)
    ament_add_pytest_test(${test_name} test/${test_name}.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
      TIMEOUT 60
    )
  endforeach()
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_visibility test/test_visibility.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_grid_patch test/test_grid_patch.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
