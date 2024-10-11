#ifndef ROS2_CONTROL_ROBOT_ARM_HPP_
#define ROS2_CONTROL_ROBOT_ARM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

namespace robot_arm_ctrl
{
class RobotArmHardwareDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotArmHardwareDriver);

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_ROBOT_ARM_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // params
  double slowdown_;
  double hw_sensor_change_;
  
  // command interface
  std::vector<double> hw_commands_;

  // state interface
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // sensors
  std::vector<double> hw_sensor_states_;
};

}

#endif