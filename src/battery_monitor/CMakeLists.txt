cmake_minimum_required(VERSION 3.8)
project(battery_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(battery_simulator src/battery_simulate.cpp)
ament_target_dependencies(battery_simulator rclcpp example_interfaces custom_interfaces)

add_executable(battery_warn src/battery_warn.cpp)
ament_target_dependencies(battery_warn rclcpp example_interfaces)

add_executable(battery_service src/battery_service.cpp)
ament_target_dependencies(battery_service rclcpp example_interfaces custom_interfaces)

install(TARGETS
      battery_simulator
      battery_warn
      battery_service
      DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
