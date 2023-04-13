cmake_minimum_required(VERSION 3.8)
project(fusion-engine-driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 14)

# Set CMake policy for CMP0077 to NEW
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# Disable example building for fusion engine client
set(P1_FE_BUILD_EXAMPLES OFF)

# Fetch and include fusion engine client library
include(FetchContent)
FetchContent_Declare(
    fusion_engine_client
    GIT_REPOSITORY https://github.com/PointOneNav/fusion-engine-client.git
    GIT_TAG v1.16.0
)
FetchContent_Populate(fusion_engine_client)
add_subdirectory(${fusion_engine_client_SOURCE_DIR} ${fusion_engine_client_BINARY_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(./core ./utils ./transport)

set(SRC
    ${SOURCE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/core/fusion_engine_node.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/core/fusion_engine_interface.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/serial_port.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/tcp_listener.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/udp_listener.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/tty_listener.cc
    )

add_executable(fusion_engine_ros_driver ${SRC})

target_link_libraries(fusion_engine_ros_driver fusion_engine_client)
ament_target_dependencies(fusion_engine_ros_driver rclcpp std_msgs gps_msgs sensor_msgs geometry_msgs visualization_msgs mavros_msgs nmea_msgs)

install(TARGETS
  fusion_engine_ros_driver
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
