cmake_minimum_required(VERSION 3.8)
project(motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_executable(test_con 
  src/test_con.cpp 
  src/motor_driver.cpp
)
ament_target_dependencies(test_con rclcpp serial)

install(TARGETS 
  test_con 
  DESTINATION lib/${PROJECT_NAME}/
)




ament_package()
