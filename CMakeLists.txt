cmake_minimum_required(VERSION 3.8)
project(esp32_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the Python interpreter
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Install the Python script
install(PROGRAMS
  src/imu_publisher.py
  src/motion_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
