cmake_minimum_required(VERSION 3.8)
project(fusion-engine-driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(./include/common)
include_directories(./include/interfaces)
include_directories(./include)
include_directories(./include/messages)
include_directories(./include/parsers)
include_directories(./include/listener)

set(SRC
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/fusion_engine_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common/logging.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/crc.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/include/parsers/fusion_engine_framer.cc
    )

add_executable(gps ${SRC})

ament_target_dependencies(gps rclcpp std_msgs gps_msgs sensor_msgs geometry_msgs visualization_msgs)


install(TARGETS
  gps
  DESTINATION lib/${PROJECT_NAME}
  )
  
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

ament_package()
