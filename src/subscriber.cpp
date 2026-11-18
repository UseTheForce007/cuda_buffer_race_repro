// One subscriber process, one single-threaded executor, several Image topics, accepting any buffer
// backend. For every message it reads the pixels and compares them with the message's own frame_id.
//
// Usage: race_subscriber <seconds> <mode> <work_ms> <topic> [<topic> ...]
//   mode: "copy"      device-to-host copy of the whole frame through the public cuda_buffer API
//         "serialize" rclcpp::Serialization of the message (what a recorder does)
//   work_ms: extra per-message delay to make the subscriber fall behind (0 = none)
#include <cuda_runtime.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "sensor_msgs/msg/image.hpp"

struct Stats
{
  size_t received{0};
  size_t empty{0};        // non-CPU buffer with zero elements (detectable stale case)
  size_t bad_size{0};
  size_t wrong_frame{0};  // pixels do not match this message's frame_id
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc < 5) {
    std::fprintf(stderr, "usage: %s seconds mode work_ms topic...\n", argv[0]);
    return 1;
  }
  const double seconds = std::stod(argv[1]);
  const std::string mode = argv[2];
  const int work_ms = std::stoi(argv[3]);
  auto node = std::make_shared<rclcpp::Node>("race_subscriber");
  rclcpp::SubscriptionOptions opts;
  opts.acceptable_buffer_backends = "any";
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  rclcpp::Serialization<sensor_msgs::msg::Image> ser;

  std::map<std::string, Stats> stats;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs;
  std::vector<uint8_t> host;
  for (int i = 4; i < argc; ++i) {
    const std::string topic = argv[i];
    stats[topic];
    subs.push_back(node->create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::QoS(10),
        [&, topic](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
          auto & s = stats[topic];
          ++s.received;
          const size_t expected = static_cast<size_t>(msg->height) * msg->step;
          if (msg->data.size() == 0) {
            if (msg->data.get_backend_type() != "cpu") {++s.empty;}
            return;
          }
          if (msg->data.size() != expected) {
            ++s.bad_size;
            return;
          }
          const uint8_t want = static_cast<uint8_t>(std::stoul(msg->header.frame_id) % 256);
          uint8_t first, mid, last;
          if (mode == "copy") {
            host.resize(expected);
            auto rh = cuda_buffer_backend::from_input_buffer(msg->data, stream);
            cudaMemcpyAsync(host.data(), rh.get_ptr(), expected, cudaMemcpyDeviceToHost, stream);
            cudaStreamSynchronize(stream);
            first = host[0]; mid = host[expected / 2]; last = host[expected - 1];
          } else {
            rclcpp::SerializedMessage out;
            ser.serialize_message(msg.get(), &out);
            const auto & rcl = out.get_rcl_serialized_message();
            const uint8_t * end = rcl.buffer + rcl.buffer_length;  // data is the last field
            first = end[-static_cast<ptrdiff_t>(expected)];
            mid = end[-1 - static_cast<ptrdiff_t>(expected / 2)];
            last = end[-1];
          }
          if (first != want || mid != want || last != want) {
            ++s.wrong_frame;
            std::printf(
              "MISMATCH %s frame_id=%s want=%u got=%u offset=+%u\n", topic.c_str(),
              msg->header.frame_id.c_str(), want, last, static_cast<uint8_t>(last - want));
          }
          if (work_ms > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(work_ms));}
        }, opts));
  }
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    exec.spin_some(std::chrono::milliseconds(20));
  }
  size_t tr = 0, te = 0, tw = 0;
  for (const auto & [topic, s] : stats) {
    std::printf(
      "SUMMARY %s received=%zu empty=%zu bad_size=%zu wrong_frame=%zu\n", topic.c_str(),
      s.received, s.empty, s.bad_size, s.wrong_frame);
    tr += s.received; te += s.empty; tw += s.wrong_frame;
  }
  std::printf("TOTAL received=%zu empty=%zu wrong_frame=%zu\n", tr, te, tw);
  cudaStreamDestroy(stream);
  rclcpp::shutdown();
  return 0;
}
