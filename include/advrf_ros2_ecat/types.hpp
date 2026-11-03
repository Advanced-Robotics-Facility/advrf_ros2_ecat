#pragma once
#include <advrf_ros2_interfaces/msg/motor.hpp>
#include <advrf_ros2_interfaces/msg/motor_tx_pdo_vector.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <api/ec_types.h>
#include <array>

namespace advrf_ros2_ecat
{

constexpr std::size_t max_motors = 128;
using Motor = advrf_ros2_interfaces::msg::Motor;
using Commands = advrf_ros2_interfaces::msg::MotorTxPdoVector;
using JointState = sensor_msgs::msg::JointState;
struct State
{
    int64_t timestamp_ns = 0;
    std::size_t size = 0;
    std::array<int32_t, max_motors> ids{};
    std::array<MotorPdoRx::pdo_t, max_motors> values{};
};

}
