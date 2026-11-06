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
  void activate_on_first_message(bool enabled) { auto_activation_pending_ = enabled; }
  bool awaiting_first_message() const { return auto_activation_pending_; }
  bool watchdog_armed() const { return active() && received_command_; }
  bool can_accept() const { return active() || awaiting_first_message(); }
  bool set_active(bool enabled, Clock::time_point = Clock::now())
  {
    auto_activation_pending_ = false; // An explicit service request takes precedence.
    if (enabled == active()) return false;
    received_command_ = false;
    active_.store(enabled);
    return true;
  }
  bool command_received(Clock::time_point now = Clock::now())
  {
    if (!can_accept()) return false;
    if (auto_activation_pending_)
    {
      auto_activation_pending_ = false;
      active_.store(true);
    }
    last_command_ = now;
    received_command_ = true;
    return true;
  }
  bool expired(std::chrono::milliseconds timeout, Clock::time_point now = Clock::now()) const
  {
    return watchdog_armed() && now - last_command_ >= timeout;
  }
  std::chrono::milliseconds command_age(Clock::time_point now = Clock::now()) const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_);
  }
private:
  std::atomic<bool> active_{false};
  bool received_command_ = false;
  bool auto_activation_pending_ = false;
  Clock::time_point last_command_{};
};
}
