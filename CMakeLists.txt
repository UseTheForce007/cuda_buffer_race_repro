cmake_minimum_required(VERSION 3.20)
project(cuda_buffer_race_repro)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(race_publisher src/publisher.cpp)
target_link_libraries(race_publisher
  rclcpp::rclcpp ${sensor_msgs_TARGETS} cuda_buffer::cuda_buffer CUDA::cudart)

add_executable(race_subscriber src/subscriber.cpp)
target_link_libraries(race_subscriber
  rclcpp::rclcpp ${sensor_msgs_TARGETS} cuda_buffer::cuda_buffer CUDA::cudart)

install(TARGETS race_publisher race_subscriber DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS run.sh DESTINATION lib/${PROJECT_NAME})
ament_package()
