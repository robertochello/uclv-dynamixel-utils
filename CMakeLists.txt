cmake_minimum_required(VERSION 3.8)
project(uclv_dynamixel_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)  


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

include_directories(include)

add_library(uclv_dynamixel_utils
  src/hand.cpp
  src/motor.cpp
  src/fingermotor.cpp
  src/wristmotor.cpp
)

ament_target_dependencies(uclv_dynamixel_utils
    "dynamixel_sdk"
)
ament_export_targets(uclv_dynamixel_utilsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(dynamixel_sdk)

install(
  TARGETS uclv_dynamixel_utils
  EXPORT uclv_dynamixel_utilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
