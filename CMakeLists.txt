cmake_minimum_required(VERSION 3.16)

project(advrf_ros2_ecat LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

set(ECAT_CLIENT_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../ecat_ws/src/ecat-client-advr"
  CACHE PATH "Upstream client source containing its matching bundled spdlog headers"
)

option(XENOMAI "Enable Xenomai support" OFF)

set(ECAT_INSTALL_PREFIX "$ENV{ECAT_INSTALL_PREFIX}"
  CACHE PATH "Prefix containing the CMake-built master and client"
)

if(ECAT_INSTALL_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${ECAT_INSTALL_PREFIX}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(advrf_ros2_interfaces REQUIRED)

find_package(Eigen3 REQUIRED)

if(XENOMAI)
  find_package(Xenomai REQUIRED)
endif()

find_package(ecat_master REQUIRED COMPONENTS ecat_udp ecat_core)
find_package(ecat_client_advr REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(advrf_ros2_ecat_lib
  src/converter.cpp
)

target_include_directories(advrf_ros2_ecat_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(advrf_ros2_ecat_lib PUBLIC
  rclcpp
  sensor_msgs
  diagnostic_msgs
  std_msgs
  std_srvs
  advrf_ros2_interfaces
)

target_link_libraries(advrf_ros2_ecat_lib PUBLIC
  ecat_client_advr::ec_library
  ecat_master::tools_utils
  Eigen3::Eigen
  Threads::Threads
  yaml-cpp
)

add_executable(bridge_ecat_client_node src/advrf_ros2_ecat.cpp)
target_link_libraries(bridge_ecat_client_node PRIVATE advrf_ros2_ecat_lib)

target_link_options(bridge_ecat_client_node PRIVATE "LINKER:--exclude-libs,ALL")

if(Xenomai_FOUND)
  xenomai_target_bootstrap(bridge_ecat_client_node SKINS posix)
endif()

set_target_properties(bridge_ecat_client_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS bridge_ecat_client_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_converter test/test_converter.cpp)
  target_link_libraries(test_converter advrf_ros2_ecat_lib)
  ament_add_gtest(test_command_gate test/test_command_gate.cpp)
  target_include_directories(test_command_gate PRIVATE include)
endif()
ament_package()
