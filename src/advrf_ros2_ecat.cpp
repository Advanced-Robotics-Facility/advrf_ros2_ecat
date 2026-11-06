#include <sched.h>
#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>
#include <pthread.h>
#include <semaphore.h>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <system_error>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "advrf_ros2_ecat/command_gate.hpp"

#include "advrf_ros2_ecat/converter.hpp"
#include "utils/ec_wrapper.h"
#include "utils/ec_thread.h"

using namespace std::chrono;

EcIface::Ptr client;
EcUtils::EC_CONFIG ec_cfg;

namespace
{
volatile std::sig_atomic_t run_loop = 1;
static pthread_mutex_t reference_mutex;
static sem_t writer_ready;
advrf_ros2_ecat::CommandGate command_gate;
bool reference_pending = false;

void sig_handler(int)
{
    run_loop = 0;
}

bool on_motor_command(const advrf_ros2_ecat::Commands &msg)
{
    if (ec_cfg.protocol == "iddp") {
        pthread_mutex_lock(&reference_mutex);
    }

    bool accepted = false;
    try
    {
        auto references = advrf_ros2_ecat::convert::from_ros(msg, motor_reference_map);
        if (references && command_gate.command_received())
        {
            motor_reference_map = std::move(*references);
            reference_pending = true;
            accepted = true;
        }
    }
    catch (...)
    {
        if (ec_cfg.protocol == "iddp") {
            pthread_mutex_unlock(&reference_mutex);
        }
        throw;
    }

    if (ec_cfg.protocol == "iddp")
    {
        pthread_mutex_unlock(&reference_mutex);
        if (accepted && sem_post(&writer_ready) != 0)
            throw std::system_error(errno, std::generic_category(), "writer_ready notification");
    }
    return accepted;
}

void set_reference()
{
    if (!motor_reference_map.empty()) {
        client->set_motor_reference(motor_reference_map);
    }

    if (!gripper_reference_map.empty()) {
        client->set_gripper_reference(gripper_reference_map);
    }
}

class EcWriter : public EcThread
{
public:

    EcWriter() 
    {
        fd_timer = -1;
        name = "advrf_ecat_wr";

        pthread_mutexattr_t reference_mutex_attr;

        pthread_mutexattr_init(&reference_mutex_attr);
        pthread_mutexattr_settype(&reference_mutex_attr, PTHREAD_MUTEX_NORMAL);
        pthread_mutexattr_setpshared(&reference_mutex_attr, PTHREAD_PROCESS_PRIVATE);
        pthread_mutexattr_setprotocol(&reference_mutex_attr, PTHREAD_PRIO_NONE);

        const int ret = pthread_mutex_init(&reference_mutex, &reference_mutex_attr);
        pthread_mutexattr_destroy(&reference_mutex_attr);

        if (ret != 0)
            throw std::system_error(ret, std::generic_category(), "reference_mutex initialization");

        if (sem_init(&writer_ready, 0, 0) != 0)
        {
            const int error = errno;
            pthread_mutex_destroy(&reference_mutex);
            throw std::system_error(error, std::generic_category(), "writer_ready initialization");
        }

        const auto client_thread_info = client->get_client_thread_info();
        schedpolicy = client_thread_info.policy;
        if (schedpolicy == SCHED_OTHER)
            priority = 0;
        else 
            priority = std::max(1, client_thread_info.priority - 1);
        period.period = {0, 1};
        stacksize = 0;
    }

    ~EcWriter() override
    {
        stop_writer();
        sem_destroy(&writer_ready);
        pthread_mutex_destroy(&reference_mutex);
    }

    void stop_writer() noexcept
    {
        active.store(false);
        sem_post(&writer_ready);
        join();
    }

    bool running() const
    {
        return active.load();
    }

    void rethrow_error() const
    {
        if (error)
            std::rethrow_exception(error);
    }

    void th_init(void *) override
    {
        // DPRINTF("Writer thread initialized, ");
        // DPRINTF("id: %ld cpu: %d, priority %d\n",pthread_self(),sched_getcpu(),priority);  
    }

    void th_loop(void *) override
    {
        try
        {
            while (run_loop && client->get_client_status().run_loop)
            {
                if (sem_wait(&writer_ready) == 0)
                {
                    if (!run_loop || !active.load())
                        break;

                    pthread_mutex_lock(&reference_mutex);
                    try
                    {
                        if (command_gate.active() && reference_pending)
                        {
                            set_reference();
                            client->write();
                            reference_pending = false;
                        }
                    }
                    catch (...)
                    {
                        pthread_mutex_unlock(&reference_mutex);
                        throw;
                    }
                    pthread_mutex_unlock(&reference_mutex);

                }
                else if (errno != EINTR)
                {
                    throw std::system_error(errno, std::generic_category(), "writer_ready wait");
                }
            }
        }
        catch (...)
        {
            error = std::current_exception();
        }

        active.store(false);
        _run_loop = 0;
    }

private:
    std::atomic<bool> active{true};
    std::exception_ptr error;
};
} 

int main(int argc, char **argv)
{
    using namespace advrf_ros2_ecat;
    try
    {
        if (argc == 2 && std::string(argv[1]) == "--help")
        {
            std::cout << "Usage: EC_CFG=/path/ec_cfg.yaml ros2 run advrf_ros2_ecat bridge_ecat_client_node "
                         "[--ros-args ...]\n";
            return 0;
        }

        const char *path = std::getenv("EC_CFG");
        if (!path || !*path)
            throw std::runtime_error("Set EC_CFG to the client YAML file");

        // ROS identity/domain come from --ros-args and ROS_DOMAIN_ID.
        // EC_CFG is consumed only by the EtherCAT client library below.
        rclcpp::init(argc, argv);

        struct RosGuard
        {
            ~RosGuard() {
                rclcpp::shutdown();
            }
        } ros_guard;

        auto node = std::make_shared<rclcpp::Node>("bridge_ecat_client_node", "/advrf/bench/ecat");

        // Read startup parameters before creating or starting the EtherCAT system.
        rcl_interfaces::msg::ParameterDescriptor startup_parameter;
        startup_parameter.read_only = true;

        const int period_ms = node->declare_parameter<int>("period_ms", 4, startup_parameter);
        const bool active_tx_on_first_msg = node->declare_parameter<bool>(
            "active_tx_on_first_msg", false, startup_parameter);
        command_gate.activate_on_first_message(active_tx_on_first_msg);

        const int command_timeout_ms = node->declare_parameter<int>(
            "command_timeout_ms", 500, startup_parameter);
        if (command_timeout_ms <= 0)
            throw std::invalid_argument("command_timeout_ms must be positive");

        if (period_ms <= 0)
            throw std::invalid_argument("period_ms must be positive");

        const auto endpoint_qos = [&](const char *endpoint)
        {
            const std::string base = std::string("qos.") + endpoint + ".";
            const auto depth = node->declare_parameter<int>(base + "depth", 1, startup_parameter);
            const auto reliability = node->declare_parameter<std::string>(
                base + "reliability", endpoint == std::string("state") ? "reliable" : "best_effort",
                startup_parameter);
            const auto durability = node->declare_parameter<std::string>(
                base + "durability", "volatile", startup_parameter);
            if (depth <= 0 || depth > 100000)
                throw std::invalid_argument("Invalid QoS depth");
            if (reliability != "reliable" && reliability != "best_effort")
                throw std::invalid_argument("Invalid QoS reliability");
            if (durability != "transient_local" && durability != "volatile")
                throw std::invalid_argument("Invalid QoS durability");
            rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(depth))};
            if (reliability == "reliable") qos.reliable();
            else qos.best_effort();
            if (durability == "transient_local") qos.transient_local();
            else qos.durability_volatile();
            return qos;
        };
        const auto state_qos = endpoint_qos("state");
        const auto command_qos = endpoint_qos("command");

        EcWrapper ec_wrapper;
        ec_wrapper.create_ec(client, ec_cfg);

        std::signal(SIGINT, sig_handler);
        std::signal(SIGTERM, sig_handler);

        // The client wrapper owns device startup, including control modes and gains.
        const bool ec_sys_started = ec_wrapper.start_ec_sys();
        if (!ec_sys_started)
            throw std::runtime_error("EtherCAT system startup failed");
        if (motor_reference_map.empty() || motor_reference_map.size() > max_motors)
            throw std::runtime_error("Missing motor references or too many motors");

        std::map<int32_t, std::string> names;
        for (const auto &[id, reference] : motor_reference_map)
        {
            const auto &config = ec_cfg.device_config_map.at(id);
            names[id] = config.device_name;
        }

        EcWriter ec_writer;
        bool command_received = false;
        using Clock = std::chrono::steady_clock;
        bool fault = false;
        std::string fault_reason;
        auto diagnostics = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(10).reliable());
        auto active_publisher = node->create_publisher<std_msgs::msg::Bool>(
            "active", rclcpp::QoS(1).reliable().transient_local());
        const auto publish_active = [&]()
        {
            std_msgs::msg::Bool state;
            state.data = command_gate.active();
            active_publisher->publish(state);
        };
        publish_active();
        const auto enter_fault = [&](const std::string &reason)
        {
            if (fault) return;
            fault = true;
            fault_reason = reason;
            command_gate.set_active(false);
            command_received = false;
            publish_active();
            // Join the writer before STOP so no queued reference can follow it.
            ec_writer.stop_writer();
            try
            {
                if (!client->stop_devices())
                    fault_reason += "; motor STOP failed";
            }
            catch (const std::exception &error)
            {
                fault_reason += std::string("; motor STOP failed: ") + error.what();
            }
            RCLCPP_ERROR(node->get_logger(), "%s", fault_reason.c_str());
        };
        const auto publish_diagnostics = [&]()
        {
            diagnostic_msgs::msg::DiagnosticArray array;
            array.header.stamp = node->now();
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = std::string(node->get_fully_qualified_name()) + ": EtherCAT";
            status.hardware_id = "EtherCAT";
            status.level = fault ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                : diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = fault ? "FAULT: " + fault_reason
                : command_gate.watchdog_armed() ? "PIPE OK - TX communicating"
                : command_gate.active() ? "PIPE OK - TX active, pending first valid command"
                : command_gate.awaiting_first_message() ? "PIPE OK - waiting for first valid command"
                : "PIPE OK - TX inactive";
            const auto value = [&](const std::string &key, const std::string &text)
            {
                diagnostic_msgs::msg::KeyValue entry;
                entry.key = key;
                entry.value = text;
                status.values.push_back(entry);
            };
            value("mode", fault ? "FAULT" : command_gate.watchdog_armed() ? "ACTIVE_COMMUNICATING"
                : command_gate.active() ? "ACTIVE_PENDING_MESSAGE"
                : command_gate.awaiting_first_message() ? "WAITING_FIRST_MESSAGE" : "INACTIVE");
            value("pipe", client->get_client_status().run_loop ? "OK" : "FAULT");
            value("active", command_gate.active() ? "true" : "false");
            value("auto_activation_pending", command_gate.awaiting_first_message() ? "true" : "false");
            value("watchdog_armed", command_gate.watchdog_armed() ? "true" : "false");
            value("command_timeout_ms", std::to_string(command_timeout_ms));
            value("watchdog_age_ms", command_gate.watchdog_armed()
                ? std::to_string(command_gate.command_age().count()) : "inactive");
            array.status.push_back(std::move(status));
            diagnostics->publish(array);
            publish_active();
        };

        auto set_active_tx = node->create_service<std_srvs::srv::SetBool>(
            "set_active_tx", [&](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            {
                if (request->data && fault)
                {
                    response->success = false;
                    response->message = "Bridge fault: restart required";
                    return;
                }
                if (ec_cfg.protocol == "iddp") pthread_mutex_lock(&reference_mutex);
                const bool changed = command_gate.set_active(request->data);
                if (changed)
                {
                    reference_pending = false;
                    command_received = false;
                }
                if (ec_cfg.protocol == "iddp") pthread_mutex_unlock(&reference_mutex);
                publish_active();
                publish_diagnostics();
                response->success = true;
                response->message = command_gate.active() ? (command_gate.watchdog_armed() ? "TX communicating; watchdog armed"
                    : "TX active; waiting for first valid command; watchdog disarmed") : "TX inactive; watchdog disarmed";
                RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
            });

        auto motors = node->create_publisher<Motor>("rx/motors", state_qos);
        auto joints = node->create_publisher<JointState>("rx/joint_states", state_qos);

        auto commands = node->create_subscription<Commands>(
            "tx/motors", command_qos,
            [&](const Commands &msg)
            {
                if (fault || !command_gate.can_accept()) return;
                const bool was_armed = command_gate.watchdog_armed();
                if (on_motor_command(msg))
                {
                    command_received = true;
                    if (!was_armed)
                    {
                        publish_active();
                        publish_diagnostics();
                        RCLCPP_INFO(node->get_logger(), "TX communicating; watchdog armed by first valid motor command");
                    }
                }
                else
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                         "Rejected invalid motor command batch");
            });

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        if (ec_cfg.protocol == "iddp")
        {
            const auto info = client->get_client_thread_info();
            ec_writer.create(info.policy != SCHED_OTHER, info.cpu >= 0 ? info.cpu : sched_getcpu());
        }

        auto time = Clock::now();
        const auto period = milliseconds(period_ms);
        auto next_diagnostic = Clock::now();
        while (run_loop && rclcpp::ok())
        {
            if (!client->get_client_status().run_loop)
                enter_fault("EtherCAT pipe stopped");
            if (!fault && !ec_writer.running())
                enter_fault("EtherCAT writer stopped");
            if (!fault && command_gate.expired(milliseconds(command_timeout_ms)))
                enter_fault("Motor command watchdog timeout");
            if (Clock::now() >= next_diagnostic || fault)
            {
                publish_diagnostics();
                next_diagnostic = Clock::now() + milliseconds(500);
            }
            if (fault)
            {
                executor.spin_some();
                std::this_thread::sleep_for(milliseconds(50));
                continue;
            }
            client->read();
            client->get_motor_status(motor_status_map);

            State state;
            state.timestamp_ns = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
            for (const auto &[id, name] : names)
            {
                state.ids[state.size] = id;
                state.values[state.size++] = motor_status_map.at(id);
            }

            // ROS 2 publishers.
            const auto telemetry = convert::to_ros(state, names, motor_reference_map);
            motors->publish(telemetry.motors);
            joints->publish(telemetry.joints);

            // ROS 2 subscriber invokes on_motor_command() only for incoming messages.
            command_received = false;
            executor.spin_some();

            if (ec_cfg.protocol != "iddp" && command_gate.active() && command_received)
            {
                set_reference();
                client->write();
            }

            ec_wrapper.log_ec_sys();

            time += period;
            if (time < Clock::now())
                time = Clock::now();
            std::this_thread::sleep_until(time);
        }

        ec_writer.stop_writer();
        ec_writer.rethrow_error();
        ec_wrapper.stop_ec_sys();
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bridge_ecat_client_node"), "%s", error.what());
        return 1;
    }

    return 0;
}
