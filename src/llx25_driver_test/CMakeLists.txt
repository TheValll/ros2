cmake_minimum_required(VERSION 3.8)
project(lx225_driver_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(lx225_test src/lx225_test.cpp)
ament_target_dependencies(lx225_test rclcpp)

install(TARGETS
  lx225_test
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
