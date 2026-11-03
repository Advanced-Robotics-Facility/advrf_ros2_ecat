#pragma once

#include <advrf_ros2_ecat/types.hpp>
#include <optional>

namespace advrf_ros2_ecat::convert
{

struct Telemetry
{
    Motor motors;
    JointState joints;
};

Telemetry to_ros(const State& state, const std::map<int32_t, std::string>& names,
                 const MotorReferenceMap& configured_references);

std::optional<MotorReferenceMap> from_ros(const Commands& message, const MotorReferenceMap& previous);

} 
