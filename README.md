# cuda_buffer_race_repro

Reproducer for a late subscriber reading the next frame from a recycled `cuda_buffer` block. It uses
only `rclcpp`, `sensor_msgs` and the public `cuda_buffer` API.

- `race_publisher`: publishes CUDA-backed `sensor_msgs/Image`. Frame N has every byte set to `N % 256`
  and `header.frame_id = "N"`.
- `race_subscriber`: one process, one single-threaded executor, several topics, accepting any backend.
  For each message it reads the pixels and compares them with that message's own `frame_id`.
  - `copy` mode: device-to-host copy of the whole frame with `cuda_buffer_backend::from_input_buffer`
    and `cudaMemcpyAsync`.
  - `serialize` mode: `rclcpp::Serialization` of the message (what a recorder does).
  - an optional per-message delay (`work_ms`) makes the subscriber fall behind.
- `run.sh`: starts the subscriber and N publisher processes and prints a summary.
- `trace/`: temporary, environment-gated logging patch for `cuda_buffer` and the script that analyzes
  it (see "Tracing").

## Build

Needs ROS 2 Rolling with `cuda_buffer` and `cuda_buffer_backend` from ros2/rosidl_buffer_backends
built in the same workspace, CUDA, and `rmw_fastrtps_cpp`.

```bash
colcon build --packages-select cuda_buffer_race_repro
source install/setup.bash
```

## Run

```bash
# run.sh [N_topics] [width] [height] [rate_hz] [seconds] [mode] [work_ms]
ros2 pkg prefix cuda_buffer_race_repro   # the script is in lib/cuda_buffer_race_repro/
$(ros2 pkg prefix cuda_buffer_race_repro)/lib/cuda_buffer_race_repro/run.sh 5 3840 2160 30 20 serialize 0
$(ros2 pkg prefix cuda_buffer_race_repro)/lib/cuda_buffer_race_repro/run.sh 5 3840 2160 30 20 copy 5
```

Output lines:

- `MISMATCH <topic> frame_id=N want=W got=G offset=+K`: the pixels belong to frame N+K, not N.
- `TOTAL received=... empty=... wrong_frame=...`: `empty` counts messages delivered with a zero-length
  non-CPU buffer (the stale UID case, which is detectable); `wrong_frame` counts the case this report
  is about.

A single topic is clean (control): `run.sh 1 3840 2160 30 20 copy 0` gives `wrong_frame=0`.

## Results on my machine

Linux 7.0.0-43-generic, ROS 2 Rolling, rmw_fastrtps_cpp, CUDA 12.3, NVIDIA RTX 3060 (6 GB), 5 topics,
3840x2160 rgb8, 30 Hz, 20 s per run:

| Run | empty (stale) | wrong_frame | offsets |
|---|---|---|---|
| control, 1 topic | 0 | 0 | |
| serialize, run 1 | 152 | 1 | +4 |
| serialize, run 2 | 132 | 1 | +4 |
| copy, work_ms=5, run 1 | 666 | 2 | +3, +3 |
| copy, work_ms=5, run 2 | 747 | 0 | |

The effect is intermittent: expect 0 to a few wrong-frame messages per run, always on the topic or
topics that fall furthest behind. I did not see it at 15 Hz (12 topics) or 20 Hz (9 topics) in six
runs with similar numbers of stale drops, so it may depend on the rate.

## Tracing

`trace/cuda_buffer_trace.patch` adds logging (enabled by `CUDA_BUFFER_TRACE=<file>`, off otherwise) to
`CudaMemoryPool::allocate()`, `CudaMemoryPool::assign_uid()` and `CudaVmmIPCManager::import_block()`.
Apply it to ros2/rosidl_buffer_backends at d7cd964, rebuild `cuda_buffer` and `cuda_buffer_backend`,
export `CUDA_BUFFER_TRACE=/tmp/trace.log` before `run.sh`, then:

```bash
python3 trace/analyze_cuda_trace.py /tmp/trace.log
```

It reports the allocate-to-assign window length and the imports that passed the UID check while the
block was allocated but not yet reassigned. In three runs of an earlier version of this setup that count
equalled the number of wrong-frame messages the subscriber saw (5, 3, 1).
