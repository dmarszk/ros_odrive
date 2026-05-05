
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_endpoint_client.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace odrive_ros2_control {

namespace {

constexpr uint16_t kSetGpioEndpointId = 702;

bool parse_bool_parameter(const std::unordered_map<std::string, std::string>& parameters, const std::string& key) {
    const auto it = parameters.find(key);
    if (it == parameters.end()) {
        return false;
    }

    return it->second == "true" || it->second == "1";
}

} // namespace

class Axis;

struct ODriveEndpointSession {
    ODriveEndpointSession(SocketCanIntf* can_intf, uint32_t node_id)
        : node_id_(node_id), endpoint_client_(can_intf, node_id) {}

    uint32_t node_id_;
    ODriveEndpointClient endpoint_client_;
};

struct GpioOutput {
    GpioOutput(std::string component_name, std::string interface_name, uint32_t node_id, uint32_t gpio_num, bool inverted)
        : component_name_(std::move(component_name)),
          interface_name_(std::move(interface_name)),
          node_id_(node_id),
          gpio_num_(gpio_num),
          inverted_(inverted) {}

    std::string component_name_;
    std::string interface_name_;
    uint32_t node_id_;
    uint32_t gpio_num_;
    bool inverted_ = false;
    double command_ = 0.0;
    double state_ = 0.0;
    double last_sent_command_ = std::numeric_limits<double>::quiet_NaN();
};

struct SensorStateValue {
    explicit SensorStateValue(std::string interface_name) : interface_name_(std::move(interface_name)) {}

    std::string interface_name_;
    double value_ = std::numeric_limits<double>::quiet_NaN();
};

struct TelemetrySensor {
    TelemetrySensor(std::string component_name, uint32_t node_id)
        : component_name_(std::move(component_name)), node_id_(node_id) {}

    std::string component_name_;
    uint32_t node_id_;
    std::vector<SensorStateValue> state_values_;

    void update_bus_measurements(double bus_voltage, double bus_current) {
        for (auto& state_value : state_values_) {
            if (state_value.interface_name_ == "bus_voltage") {
                state_value.value_ = bus_voltage;
            } else if (state_value.interface_name_ == "bus_current") {
                state_value.value_ = bus_current;
            }
        }
    }
};

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(const Axis& axis);
    ODriveEndpointSession* find_endpoint_session(uint32_t node_id);
    const GpioOutput* find_gpio_output(const std::string& component_name, const std::string& interface_name) const;
    void ensure_endpoint_session(uint32_t node_id);

    bool active_;
    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::vector<ODriveEndpointSession> endpoint_sessions_;
    std::vector<GpioOutput> gpio_outputs_;
    std::vector<TelemetrySensor> telemetry_sensors_;
    std::string can_intf_name_;
    bool hold_position_on_init_ = false;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id, double gear_ratio)
        : can_intf_(can_intf), node_id_(node_id), gear_ratio_(gear_ratio) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;
    double gear_ratio_;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    // uint32_t axis_error_ = 0;
    // uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    bool init_position_held = false;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    // double iq_setpoint_ = NAN;
    // double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    // uint32_t active_errors_ = 0;
    // uint32_t disarm_reason_ = 0;
    // double fet_temperature_ = NAN;
    // double motor_temperature_ = NAN;
    // double bus_voltage_ = NAN;
    // double bus_current_ = NAN;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];
    hold_position_on_init_ = info_.hardware_parameters.count("hold_position_on_init") > 0 ?
        (info_.hardware_parameters["hold_position_on_init"] == "true") : false;

    for (auto& joint : info_.joints) {
        double gear_ratio = 1.0;
        if (joint.parameters.count("gear_ratio") > 0) {
            gear_ratio = std::stod(joint.parameters.at("gear_ratio"));
        }
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")), gear_ratio);
    }

    for (const auto& gpio : info_.gpios) {
        if (gpio.parameters.count("node_id") == 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "GPIO component %s is missing required node_id parameter",
                gpio.name.c_str()
            );
            return CallbackReturn::ERROR;
        }

        const uint32_t node_id = static_cast<uint32_t>(std::stoul(gpio.parameters.at("node_id")));
        ensure_endpoint_session(node_id);

        for (const auto& command_interface : gpio.command_interfaces) {
            const auto gpio_num_it = command_interface.parameters.find("gpio_num");
            if (gpio_num_it == command_interface.parameters.end()) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("ODriveHardwareInterface"),
                    "GPIO interface %s/%s is missing required gpio_num parameter",
                    gpio.name.c_str(),
                    command_interface.name.c_str()
                );
                return CallbackReturn::ERROR;
            }

            gpio_outputs_.emplace_back(
                gpio.name,
                command_interface.name,
                node_id,
                static_cast<uint32_t>(std::stoul(gpio_num_it->second)),
                parse_bool_parameter(command_interface.parameters, "inverted")
            );
        }
    }

    for (const auto& sensor : info_.sensors) {
        if (sensor.parameters.count("node_id") == 0) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Sensor component %s is missing required node_id parameter",
                sensor.name.c_str()
            );
            return CallbackReturn::ERROR;
        }

        TelemetrySensor telemetry_sensor(sensor.name, static_cast<uint32_t>(std::stoul(sensor.parameters.at("node_id"))));
        for (const auto& state_interface : sensor.state_interfaces) {
            telemetry_sensor.state_values_.emplace_back(state_interface.name);
        }
        telemetry_sensors_.push_back(std::move(telemetry_sensor));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }
    for (auto& endpoint_session : endpoint_sessions_) {
        endpoint_session.endpoint_client_.invalidate_address();
    }
    for (auto& gpio_output : gpio_outputs_) {
        gpio_output.last_sent_command_ = std::numeric_limits<double>::quiet_NaN();
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
    }

    for (const auto& gpio : info_.gpios) {
        for (const auto& state_interface : gpio.state_interfaces) {
            if (const GpioOutput* gpio_output = find_gpio_output(gpio.name, state_interface.name)) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    gpio.name,
                    state_interface.name,
                    const_cast<double*>(&gpio_output->state_)
                ));
            }
        }
    }

    for (auto& telemetry_sensor : telemetry_sensors_) {
        for (auto& state_value : telemetry_sensor.state_values_) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                telemetry_sensor.component_name_,
                state_value.interface_name_,
                &state_value.value_
            ));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    for (auto& gpio_output : gpio_outputs_) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio_output.component_name_,
            gpio_output.interface_name_,
            &gpio_output.command_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}};

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (auto& axis : axes_) {
        if (hold_position_on_init_ && !axis.init_position_held) {
            // Hold current position until first update received
            if (std::isnan(axis.pos_estimate_)) {
                RCLCPP_WARN(
                    rclcpp::get_logger("ODriveHardwareInterface"),
                    "Axis %d: waiting for first position estimate...",
                    axis.node_id_
                );
                continue;
            } else {
                RCLCPP_INFO(
                    rclcpp::get_logger("ODriveHardwareInterface"),
                    "Axis %d: received first position estimate, holding position at %.3f rad",
                    axis.node_id_,
                    axis.pos_estimate_
                );
                axis.pos_setpoint_ = axis.pos_estimate_;
                axis.vel_setpoint_ = 0.0;
                axis.torque_setpoint_ = 0.0;
                axis.init_position_held = true;
            }
        }
        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = (axis.pos_setpoint_ / (2 * M_PI)) * axis.gear_ratio_;
            msg.Vel_FF = axis.vel_input_enabled_ ? ((axis.vel_setpoint_ / (2 * M_PI)) * axis.gear_ratio_) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? (axis.torque_setpoint_ / axis.gear_ratio_) : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = (axis.vel_setpoint_ / (2 * M_PI)) * axis.gear_ratio_;
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? (axis.torque_setpoint_ / axis.gear_ratio_) : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_ / axis.gear_ratio_;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    for (auto& gpio_output : gpio_outputs_) {
        const double requested_command = gpio_output.command_ >= 0.5 ? 1.0 : 0.0;
        if (!std::isnan(gpio_output.last_sent_command_) && requested_command == gpio_output.last_sent_command_) {
            gpio_output.state_ = requested_command;
            continue;
        }

        ODriveEndpointSession* endpoint_session = find_endpoint_session(gpio_output.node_id_);
        if (endpoint_session == nullptr) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "No endpoint session configured for GPIO %s/%s on node %u",
                gpio_output.component_name_.c_str(),
                gpio_output.interface_name_.c_str(),
                gpio_output.node_id_
            );
            return return_type::ERROR;
        }

        const bool hardware_state = gpio_output.inverted_ ? (requested_command < 0.5) : (requested_command >= 0.5);
        std::array<uint8_t, 5> payload = {};
        const uint32_t gpio_num = gpio_output.gpio_num_;
        std::memcpy(payload.data(), &gpio_num, sizeof(gpio_num));
        payload[4] = hardware_state ? 1U : 0U;

        if (!endpoint_session->endpoint_client_.call_function(kSetGpioEndpointId, payload)) {
            RCLCPP_ERROR(
                rclcpp::get_logger("ODriveHardwareInterface"),
                "Failed to send GPIO command for %s/%s on node %u",
                gpio_output.component_name_.c_str(),
                gpio_output.interface_name_.c_str(),
                gpio_output.node_id_
            );
            return return_type::ERROR;
        }

        gpio_output.last_sent_command_ = requested_command;
        gpio_output.state_ = requested_command;
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    if ((frame.can_id & 0x1f) == Get_Bus_Voltage_Current_msg_t::cmd_id) {
        if (frame.can_dlc < Get_Bus_Voltage_Current_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", frame.can_id & 0x1f);
        } else {
            Get_Bus_Voltage_Current_msg_t msg;
            msg.decode_buf(frame.data);
            const uint32_t node_id = frame.can_id >> 5;
            for (auto& telemetry_sensor : telemetry_sensors_) {
                if (telemetry_sensor.node_id_ == node_id) {
                    telemetry_sensor.update_bus_measurements(msg.Bus_Voltage, msg.Bus_Current);
                }
            }
        }
    }

    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

ODriveEndpointSession* ODriveHardwareInterface::find_endpoint_session(uint32_t node_id) {
    auto it = std::find_if(endpoint_sessions_.begin(), endpoint_sessions_.end(), [node_id](const ODriveEndpointSession& session) {
        return session.node_id_ == node_id;
    });
    if (it == endpoint_sessions_.end()) {
        return nullptr;
    }
    return &(*it);
}

const GpioOutput* ODriveHardwareInterface::find_gpio_output(
    const std::string& component_name,
    const std::string& interface_name
) const {
    auto it = std::find_if(gpio_outputs_.begin(), gpio_outputs_.end(), [&](const GpioOutput& gpio_output) {
        return gpio_output.component_name_ == component_name && gpio_output.interface_name_ == interface_name;
    });
    if (it == gpio_outputs_.end()) {
        return nullptr;
    }
    return &(*it);
}

void ODriveHardwareInterface::ensure_endpoint_session(uint32_t node_id) {
    if (find_endpoint_session(node_id) == nullptr) {
        endpoint_sessions_.emplace_back(&can_intf_, node_id);
    }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    Clear_Errors_msg_t clear_error_msg;
    Set_Axis_State_msg_t state_msg;

    clear_error_msg.Identify = 0;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "No control mode specified. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(state_msg);
        return;
    }

    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = (msg.Pos_Estimate * (2 * M_PI)) / gear_ratio_;
                vel_estimate_ = (msg.Vel_Estimate * (2 * M_PI)) / gear_ratio_;
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target * gear_ratio_;
                torque_estimate_ = msg.Torque_Estimate * gear_ratio_;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
