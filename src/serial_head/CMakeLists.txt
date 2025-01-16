cmake_minimum_required(VERSION 3.8)
project(serial_head)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)


# 编译和链接其他依赖库
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(serial_eye_ear src/serial_eye_ear.cpp)
ament_target_dependencies(serial_eye_ear rclcpp sensor_msgs serial)

target_link_libraries(serial_eye_ear SDL2)

install(TARGETS
  serial_eye_ear
  DESTINATION lib/${PROJECT_NAME}
  )
ament_package()
