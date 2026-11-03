cmake_minimum_required(VERSION 3.8)
project(luna_guidance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installs the luna_guidance/ python package so `from luna_guidance... import ...` works.
ament_python_install_package(${PROJECT_NAME})

# One entry per runnable node: scripts/<name>.py -> `ros2 run luna_guidance <name>`
set(PY_EXECUTABLES
  guidance_controller
  path_planner
)

foreach(exe ${PY_EXECUTABLES})
  install(PROGRAMS scripts/${exe}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${exe})
endforeach()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  foreach(test_name
    test_theta_star
    test_grid
    test_planner_pipeline
    test_path_planner_node
    test_clearance_planning
    test_hybrid_planner
  )
    ament_add_pytest_test(${test_name}
      test/${test_name}.py
      TIMEOUT 60
    )
  endforeach()
  find_package(ament_lint_auto REQUIRED)
  # skips the copyright linter until license headers are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
