cmake_minimum_required(VERSION 3.8)
project(learn_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs example_interfaces)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp example_interfaces)

add_executable(counterpub src/counter_publisher.cpp)
ament_target_dependencies(counterpub rclcpp example_interfaces)

add_executable(countersub src/counter_subscriber.cpp)
ament_target_dependencies(countersub rclcpp example_interfaces)

add_executable(add_two_ints_service src/simple_service.cpp)
ament_target_dependencies(add_two_ints_service rclcpp example_interfaces)

add_executable(param_change src/param_change.cpp)
ament_target_dependencies(param_change rclcpp example_interfaces)

add_executable(multiple_publisher src/multiple_publisher.cpp)
ament_target_dependencies(multiple_publisher rclcpp example_interfaces)

install(TARGETS
      publisher
      subscriber
      counterpub
      countersub
      add_two_ints_service
      param_change
      multiple_publisher
      DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
