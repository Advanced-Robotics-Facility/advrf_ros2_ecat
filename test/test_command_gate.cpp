#include <gtest/gtest.h>
#include "advrf_ros2_ecat/command_gate.hpp"
using advrf_ros2_ecat::CommandGate;
using namespace std::chrono_literals;

TEST(CommandGate, InactiveIgnoresCommandsAndHasNoWatchdog)
{
  CommandGate gate;
  const auto t = CommandGate::Clock::time_point{};
  EXPECT_FALSE(gate.active());
  gate.command_received(t);
  EXPECT_FALSE(gate.expired(500ms, t + 10s));
}
TEST(CommandGate, ActivationStartsDeadlineBeforeFirstCommand)
{
  CommandGate gate;
  const auto t = CommandGate::Clock::time_point{};
  EXPECT_TRUE(gate.set_active(true, t));
  EXPECT_FALSE(gate.expired(500ms, t + 499ms));
  EXPECT_TRUE(gate.expired(500ms, t + 500ms));
  EXPECT_FALSE(gate.set_active(true, t + 600ms));
  EXPECT_TRUE(gate.expired(500ms, t + 600ms)); // Repeated enable cannot postpone expiry.
}
TEST(CommandGate, ValidCommandRefreshesOnlyWhileActive)
{
  CommandGate gate;
  const auto t = CommandGate::Clock::time_point{};
  gate.set_active(true, t);
  gate.command_received(t + 400ms);
  EXPECT_FALSE(gate.expired(500ms, t + 899ms));
  EXPECT_TRUE(gate.expired(500ms, t + 900ms));
  gate.set_active(false, t + 900ms);
  gate.command_received(t + 1s);
  EXPECT_FALSE(gate.expired(500ms, t + 20s));
  gate.set_active(true, t + 20s);
  EXPECT_FALSE(gate.expired(500ms, t + 20499ms));
  EXPECT_TRUE(gate.expired(500ms, t + 20500ms));
}

TEST(CommandGate, AutomaticActivationWaitsWithoutWatchdog)
{
  CommandGate gate;
  const auto t = CommandGate::Clock::time_point{};
  gate.activate_on_first_message(true);
  EXPECT_FALSE(gate.active());
  EXPECT_TRUE(gate.can_accept());
  EXPECT_TRUE(gate.awaiting_first_message());
  EXPECT_FALSE(gate.expired(500ms, t + 10s));
  EXPECT_TRUE(gate.command_received(t + 10s));
  EXPECT_TRUE(gate.active());
  EXPECT_FALSE(gate.awaiting_first_message());
  EXPECT_FALSE(gate.expired(500ms, t + 10499ms));
  EXPECT_TRUE(gate.expired(500ms, t + 10500ms));
}
TEST(CommandGate, ExplicitDisableOverridesAutomaticActivation)
{
  CommandGate gate;
  const auto t = CommandGate::Clock::time_point{};
  gate.activate_on_first_message(true);
  gate.set_active(false, t);
  EXPECT_FALSE(gate.awaiting_first_message());
  EXPECT_FALSE(gate.command_received(t + 1s));
  gate.activate_on_first_message(true);
  EXPECT_TRUE(gate.command_received(t + 2s));
  gate.set_active(false, t + 3s);
  EXPECT_FALSE(gate.command_received(t + 4s));
  EXPECT_FALSE(gate.active());
  EXPECT_TRUE(gate.set_active(true, t + 5s));
}
