cmake_minimum_required(VERSION 3.5)
project(state_controller)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include ${OpenSSL_INCLUDE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(OpenSSL REQUIRED)



add_executable(state_controller src/state_controller.cpp)
target_link_libraries(state_controller OpenSSL::Crypto)
ament_target_dependencies(state_controller rclcpp std_msgs lart_msgs OpenSSL)


install(TARGETS
  state_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
