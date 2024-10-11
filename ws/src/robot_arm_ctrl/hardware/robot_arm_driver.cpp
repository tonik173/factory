#include "include/robot_arm_driver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_arm_ctrl
{

hardware_interface::CallbackReturn RobotArmHardwareDriver::on_init(const hardware_interface::HardwareInfo & info)
{
  // calls init on super class
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read params
  slowdown_ = stod(info_.hardware_parameters["slowdown"]);
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "slowdown = %f", slowdown_);
  hw_sensor_change_ = stod(info_.hardware_parameters["max_sensor_change"]);
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "max_sensor_change = %f", hw_sensor_change_);

  // initializes the vectors for the command and state interfaces with a single double with value NaN
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // COMMAND INTERFACE

    // check if there is exact one INTERFACE entry
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotArmHardwareDriver"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check if INTERFACE name == "position"
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotArmHardwareDriver"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }


    // STATE INTERFACES

    // check if there is exact one STATE entry
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotArmHardwareDriver"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // check if STATE name == "position" or "velocity"
    bool hasStatePositioninterface = false;
    bool hasStateVelocityinterface = false;
    for (uint i = 0; i < joint.state_interfaces.size(); i++)
    {
      hasStatePositioninterface |= joint.state_interfaces[i].name != hardware_interface::HW_IF_POSITION;
      hasStateVelocityinterface |= joint.state_interfaces[i].name != hardware_interface::HW_IF_VELOCITY;
    }

    if (!(hasStatePositioninterface && hasStateVelocityinterface))
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Joint name: %s", joint.name.c_str());
      for (uint i = 0; i < joint.state_interfaces.size(); i++)
      {
        RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), 
          "State interface name: %s. Should have 'velocity' and 'position'", joint.state_interfaces[i].name.c_str());
      }
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Successfully inited!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmHardwareDriver::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  if (std::isnan(hw_sensor_states_[0]))
  {
     hw_sensor_states_[0] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotArmHardwareDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Successfully collected state interfaces!");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotArmHardwareDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Successfully collected command interfaces!");
  return command_interfaces;
}

// -----------------------------------------------------------------------------------
// Activation
// -----------------------------------------------------------------------------------
hardware_interface::CallbackReturn RobotArmHardwareDriver::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (uint i = 0; i < hw_positions_.size(); i++)
  {
    hw_commands_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------------
// Deactivation
// -----------------------------------------------------------------------------------
hardware_interface::CallbackReturn RobotArmHardwareDriver::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------------
// Reading
// -----------------------------------------------------------------------------------
hardware_interface::return_type RobotArmHardwareDriver::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Since we don't have a real robot, we can't read the position and velocity.
  // We pretend as if the values where read from the hardware
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Reading...");

  for (uint i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = hw_positions_[i] + (hw_commands_[i] - hw_positions_[i]) / slowdown_;
    RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Got state %.5f for joint %d!", hw_positions_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Joints successfully read.");

  // Same here: we don't have a sensor. We return a random value.
  for (uint i = 0; i < hw_sensor_states_.size(); i++)
  {
    unsigned int seed = time(NULL) + i;
    hw_sensor_states_[i] = static_cast<float>(rand_r(&seed)) / (static_cast<float>(RAND_MAX / hw_sensor_change_));
    RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Got value %e for interface %s!",
        hw_sensor_states_[i], info_.sensors[0].state_interfaces[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Sensors successfully read!");

  return hardware_interface::return_type::OK;
}

// -----------------------------------------------------------------------------------
// Writing
// -----------------------------------------------------------------------------------
hardware_interface::return_type RobotArmHardwareDriver::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Got command %.5f for joint %d!", hw_commands_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareDriver"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_arm_ctrl::RobotArmHardwareDriver, hardware_interface::SystemInterface)