// Publishes CUDA-backed sensor_msgs/Image. Frame N has every byte set to (N % 256) and
// header.frame_id set to the decimal string of N, so a subscriber can tell which frame's pixels it got.
// Usage: race_publisher <topic> <width> <height> <rate_hz> <seconds> [sync]
// With "sync", the fill is synchronized (cudaStreamSynchronize) before publish().
#include <cuda_runtime.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc < 6) {
    std::fprintf(stderr, "usage: %s topic width height rate_hz seconds\n", argv[0]);
    return 1;
  }
  const std::string topic = argv[1];
  const size_t w = std::stoul(argv[2]), h = std::stoul(argv[3]);
  const double rate = std::stod(argv[4]);
  const double seconds = std::stod(argv[5]);
  const bool sync = argc > 6 && std::string(argv[6]) == "sync";
  const size_t bytes = w * h * 3;

  auto node = std::make_shared<rclcpp::Node>("race_publisher");
  auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::QoS(10));
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // Let discovery finish so the subscriber is matched before the first frame.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / rate));
  const size_t total = static_cast<size_t>(rate * seconds);
  auto next = std::chrono::steady_clock::now();
  for (size_t i = 0; i < total && rclcpp::ok(); ++i) {
    sensor_msgs::msg::Image msg;
    msg.data = cuda_buffer_backend::allocate_buffer(bytes);
    msg.header.frame_id = std::to_string(i);
    msg.height = h;
    msg.width = w;
    msg.encoding = "rgb8";
    msg.step = w * 3;
    auto wh = cuda_buffer_backend::from_output_buffer(msg.data, stream);
    cudaMemsetAsync(wh.get_ptr(), static_cast<int>(i % 256), bytes, stream);
    if (sync) {cudaStreamSynchronize(stream);}
    pub->publish(msg);
    next += period;
    std::this_thread::sleep_until(next);
  }
  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);
  std::printf("published %zu frames on %s\n", total, topic.c_str());
  rclcpp::shutdown();
  return 0;
}
