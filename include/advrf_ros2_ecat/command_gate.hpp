#pragma once
#include <atomic>
#include <chrono>

namespace advrf_ros2_ecat
{
// State transitions and timestamps belong to the ROS executor. The writer only reads active().
class CommandGate
{
public:
  using Clock = std::chrono::steady_clock;
  bool active() const { return active_.load(); }
  bool set_active(bool enabled, Clock::time_point now = Clock::now())
  {
    if (enabled == active()) return false;
    last_command_ = now;
    active_.store(enabled);
    return true;
  }
  void command_received(Clock::time_point now = Clock::now())
  {
    if (active()) last_command_ = now;
  }
  bool expired(std::chrono::milliseconds timeout, Clock::time_point now = Clock::now()) const
  {
    return active() && now - last_command_ >= timeout;
  }
  std::chrono::milliseconds command_age(Clock::time_point now = Clock::now()) const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_);
  }
private:
  std::atomic<bool> active_{false};
  Clock::time_point last_command_{};
};
}
