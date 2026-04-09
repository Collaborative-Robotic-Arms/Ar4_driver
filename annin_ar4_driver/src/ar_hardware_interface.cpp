#include <annin_ar4_driver/ar_hardware_interface.hpp>
#include <cmath>
#include <sstream>

namespace annin_ar4_driver {

namespace {

std::string formatJointVectorDeg(const std::vector<double>& values) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << " | ";
    }
    oss << "j" << (i + 1) << "=" << values[i];
  }
  return oss.str();
}

}  // namespace

hardware_interface::CallbackReturn ARHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  init_variables();

  // init motor driver
  std::string serial_port = info_.hardware_parameters.at("serial_port");
  std::string ar_model = info_.hardware_parameters.at("ar_model");
  std::string velocity_control_p =
      info_.hardware_parameters.at("velocity_control_enabled");
  bool velocity_control_enabled =
      velocity_control_p == "True" || velocity_control_p == "true";
  int baud_rate = 115200;
  bool success = driver_.init(ar_model, serial_port, baud_rate,
                              info_.joints.size(), velocity_control_enabled);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // calibrate joints if needed
  bool calibrate = info_.hardware_parameters.at("calibrate") == "True";
  if (calibrate) {
    // run calibration
    RCLCPP_INFO(logger_, "Running joint calibration...");
    std::string calib_sequence = info_.hardware_parameters.at("calib_sequence");
    if (calib_sequence.length() != 7) {
      RCLCPP_ERROR(logger_, "Invalid calib_sequence length: %zu. Expected: 7",
                   calib_sequence.length());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!driver_.calibrateJoints(calib_sequence)) {
      RCLCPP_INFO(logger_, "calibration failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(logger_, "calibration succeeded.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ARHardwareInterface::init_variables() {
  // resize vectors
  int num_joints = info_.joints.size();
  actuator_pos_commands_.resize(num_joints);
  actuator_vel_commands_.resize(num_joints);
  actuator_positions_.resize(num_joints);
  actuator_velocities_.resize(num_joints);
  filtered_actuator_positions_.resize(num_joints);
  joint_positions_.resize(num_joints);
  joint_velocities_.resize(num_joints);
  joint_efforts_.resize(num_joints);
  joint_position_commands_.resize(num_joints);
  joint_velocity_commands_.resize(num_joints);
  joint_effort_commands_.resize(num_joints);
  motion_reference_actuator_pos_commands_.resize(num_joints);
  joint_offsets_.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    joint_offsets_[i] =
        std::stod(info_.joints[i].parameters["position_offset"]);
  }
}

hardware_interface::CallbackReturn ARHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // Reset Estop (if any)
  bool success = driver_.resetEStop();
  if (!success) {
    RCLCPP_ERROR(logger_,
                 "Cannot activate. Hardware E-stop state cannot be reset.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed state and command buffers from the measured robot pose so the
  // controller does not start from stale command values.
  driver_.getJointPositions(actuator_positions_);
  driver_.getJointVelocities(actuator_velocities_);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
    joint_velocities_[i] = degToRad(actuator_velocities_[i]);
    joint_position_commands_[i] = joint_positions_[i];
    joint_velocity_commands_[i] = 0.0;
    actuator_pos_commands_[i] = actuator_positions_[i];
    actuator_vel_commands_[i] = 0.0;
    motion_reference_actuator_pos_commands_[i] = actuator_positions_[i];
    filtered_actuator_positions_[i] = actuator_positions_[i];
  }
  state_filter_seeded_ = true;
  motion_log_active_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ARHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position",
                                  &joint_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                  &joint_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position",
                                    &joint_position_commands_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                    &joint_velocity_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type ARHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  driver_.getJointPositions(actuator_positions_);
  driver_.getJointVelocities(actuator_velocities_);

  if (!state_filter_seeded_) {
    filtered_actuator_positions_ = actuator_positions_;
    state_filter_seeded_ = true;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const double raw_position = actuator_positions_[i];
    const double raw_velocity = actuator_velocities_[i];
    const double previous_position = filtered_actuator_positions_[i];

    double filtered_position = raw_position;
    double filtered_velocity = raw_velocity;
    const bool at_rest =
        std::abs(raw_velocity) < kRestVelocityThresholdDeg &&
        std::abs(raw_position - previous_position) < kRestPositionDeadbandDeg;

    if (at_rest) {
      filtered_position = previous_position;
      filtered_velocity = 0.0;
    } else {
      filtered_actuator_positions_[i] = raw_position;
    }

    // apply offsets, convert from deg to rad for moveit
    joint_positions_[i] = degToRad(filtered_position + joint_offsets_[i]);
    joint_velocities_[i] = degToRad(filtered_velocity);
  }

  if (motion_log_active_) {
    bool motion_finished = true;
    std::vector<double> measured_positions_with_offsets_deg(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      measured_positions_with_offsets_deg[i] =
          actuator_positions_[i] + joint_offsets_[i];
      const bool velocity_settled =
          std::abs(actuator_velocities_[i]) < kMotionEndVelocityThresholdDeg;
      const bool position_settled =
          std::abs(actuator_pos_commands_[i] - actuator_positions_[i]) <
          kMotionEndPositionThresholdDeg;
      if (!velocity_settled || !position_settled) {
        motion_finished = false;
      }
    }

    if (motion_finished) {
      std::vector<double> target_positions_with_offsets_deg(info_.joints.size());
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        target_positions_with_offsets_deg[i] =
            actuator_pos_commands_[i] + joint_offsets_[i];
      }

      RCLCPP_INFO(logger_, "Motion end measured joints (deg): %s",
                  formatJointVectorDeg(measured_positions_with_offsets_deg).c_str());
      RCLCPP_INFO(logger_, "Motion end target joints (deg): %s",
                  formatJointVectorDeg(target_positions_with_offsets_deg).c_str());

      motion_reference_actuator_pos_commands_ = actuator_pos_commands_;
      motion_log_active_ = false;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // convert from rad to deg, apply offsets
    actuator_pos_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
    actuator_vel_commands_[i] = radToDeg(joint_velocity_commands_[i]);
  }

  bool motion_started = false;
  if (!motion_log_active_) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (std::abs(actuator_pos_commands_[i] - motion_reference_actuator_pos_commands_[i]) >
          kMotionStartThresholdDeg) {
        motion_started = true;
        break;
      }
    }
  }

  if (motion_started) {
    std::vector<double> measured_positions_with_offsets_deg(info_.joints.size());
    std::vector<double> target_positions_with_offsets_deg(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      measured_positions_with_offsets_deg[i] =
          actuator_positions_[i] + joint_offsets_[i];
      target_positions_with_offsets_deg[i] =
          actuator_pos_commands_[i] + joint_offsets_[i];
    }

    RCLCPP_INFO(logger_, "Motion start measured joints (deg): %s",
                formatJointVectorDeg(measured_positions_with_offsets_deg).c_str());
    RCLCPP_INFO(logger_, "Motion start target joints (deg): %s",
                formatJointVectorDeg(target_positions_with_offsets_deg).c_str());

    motion_log_active_ = true;
  }

  driver_.update(actuator_pos_commands_, actuator_vel_commands_,
                 actuator_positions_, actuator_velocities_);
  if (driver_.isEStopped()) {
    std::string logWarn =
        "Hardware in EStop state. To reset the EStop "
        "reactivate the hardware component using 'ros2 "
        "run annin_ar4_driver reset_estop.sh <ar_model>'.";
    RCLCPP_WARN(logger_, logWarn.c_str());

    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace annin_ar4_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(annin_ar4_driver::ARHardwareInterface,
                       hardware_interface::SystemInterface)
