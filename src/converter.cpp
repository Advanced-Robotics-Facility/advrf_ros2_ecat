#include <advrf_ros2_ecat/converter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace advrf_ros2_ecat::convert
{
namespace
{
using MotorCommand = advrf_ros2_interfaces::msg::MotorTxPdo;

bool finite_references(const MotorCommand& message)
{
    for (float value :
         {message.pos_ref, message.vel_ref, message.tor_ref, message.cur_ref, message.gain_0,
          message.gain_1, message.gain_2, message.gain_3, message.gain_4})
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }

    return true;
}

std::optional<MotorPdoTx::pdo_t> motor_reference_from_ros(const MotorCommand& message, int configured_mode)
{
    if (message.fault_ack != 0 || message.ts != 0 || message.op_idx_aux != 0 || message.aux != 0 ||
        !finite_references(message))
    {
        return std::nullopt;
    }

    const bool current_mode = configured_mode == iit::advr::Gains_Type_CURRENT;
    if ((current_mode && message.tor_ref != 0) || (!current_mode && message.cur_ref != 0)) {
        return std::nullopt;
    }

    const float effort = current_mode ? message.cur_ref : message.tor_ref;
    float gain_0 = message.gain_0;
    float gain_1 = message.gain_1;
    float gain_2 = message.gain_2;
    float gain_3 = message.gain_3;
    float gain_4 = message.gain_4;

    if (configured_mode == iit::advr::Gains_Type_POSITION || configured_mode == iit::advr::Gains_Type_VELOCITY)
    {
        if (message.gain_2 != 0 || message.gain_3 != 0) {
            return std::nullopt;
        }
        gain_1 = message.gain_4;
        gain_2 = message.gain_1;
        gain_3 = 0;
        gain_4 = 0;
    }

    return MotorPdoTx::pdo_t{configured_mode,
                             message.pos_ref,
                             message.vel_ref,
                             effort,
                             gain_0,
                             gain_1,
                             gain_2,
                             gain_3,
                             gain_4,
                             iit::advr::AuxPDO_Op_NOP,
                             0,
                             0};
}

template <typename Status> std::string fault_report(const Status& status)
{
    if constexpr (std::tuple_size<Status>::value == 16)
    {
        const auto& report = std::get<10>(status);
        return std::string(report.begin(), std::find(report.begin(), report.end(), '\0'));
    }
    else
    {
        return {};
    }
}

} 

std::optional<MotorReferenceMap> from_ros(const Commands& message, const MotorReferenceMap& previous)
{
    if (message.data.empty() || message.data.size() > max_motors || previous.size() > max_motors) {
        return std::nullopt;
    }

    MotorReferenceMap result = previous;
    std::set<int32_t> seen;

    for (const auto& command : message.data)
    {
        auto reference = result.find(command.ecat_id);
        if (reference == result.end() || !seen.insert(command.ecat_id).second) {
            return std::nullopt;
        }

        const int configured_mode = std::get<0>(reference->second);
        auto converted = motor_reference_from_ros(command, configured_mode);
        if (!converted) {
            return std::nullopt;
        }

        reference->second = *converted;
    }

    return result;
}

Telemetry to_ros(const State& state, const std::map<int32_t, std::string>& names,
                 const MotorReferenceMap& configured_references)
{
    if (state.size > max_motors || state.timestamp_ns < 0)
    {
        throw std::invalid_argument("Invalid client state snapshot");
    }

    Telemetry result;
    auto& motors = result.motors;
    auto& joints = result.joints;
    motors.header.stamp.sec = static_cast<int32_t>(state.timestamp_ns / 1000000000);
    motors.header.stamp.nanosec = static_cast<uint32_t>(state.timestamp_ns % 1000000000);
    joints.header = motors.header;

    for (std::size_t i = 0; i < state.size; ++i)
    {
        const auto id = state.ids[i];
        const auto& name = names.at(id);
        const auto& value = state.values[i];
        constexpr std::size_t fields = std::tuple_size<MotorPdoRx::pdo_t>::value;
        static_assert(fields == 15 || fields == 16, "Unsupported motor status layout");
        constexpr std::size_t offset = fields == 16 ? 1 : 0;
        const auto& status_word = std::get<0>(value);
        const auto& link_position = std::get<1>(value);
        const auto& motor_position = std::get<2>(value);
        const auto& link_velocity = std::get<3>(value);
        const auto& motor_velocity = std::get<4>(value);
        const auto& torque = std::get<5>(value);
        const auto& current = std::get<6>(value);
        const auto& motor_temperature = std::get<7>(value);
        const auto& board_temperature = std::get<8>(value);
        const auto& fault = std::get<9>(value);
        const auto& rtt = std::get<10 + offset>(value);
        const auto& position_feedback = std::get<11 + offset>(value);
        const auto& velocity_feedback = std::get<12 + offset>(value);
        const auto& torque_feedback = std::get<13 + offset>(value);
        const auto& current_feedback = std::get<14 + offset>(value);

        motors.name.push_back(name);
        motors.statusword.push_back(status_word);
        motors.modes_of_op.push_back(std::get<0>(configured_references.at(id)));
        motors.link_pos.push_back(link_position);
        motors.motor_pos.push_back(motor_position);
        motors.link_vel.push_back(link_velocity);
        motors.motor_vel.push_back(motor_velocity);
        motors.torque.push_back(torque);
        motors.current.push_back(current);
        motors.motor_temp.push_back(motor_temperature);
        motors.drive_temp.push_back(board_temperature);
        motors.fault.push_back(fault);
        motors.error_code.push_back(fault);
        motors.error_report.push_back(fault_report(value));
        motors.rtt.push_back(rtt);
        motors.demanded_pos.push_back(position_feedback);
        motors.demanded_vel.push_back(velocity_feedback);
        motors.demanded_torque.push_back(torque_feedback);
        motors.demanded_current.push_back(current_feedback);
        //motors.control_effort.push_back(0); // Not exposed by ec_client.

        joints.name.push_back(name);
        joints.position.push_back(link_position);
        joints.velocity.push_back(link_velocity);
        joints.effort.push_back(torque);
    }

    return result;
}
}