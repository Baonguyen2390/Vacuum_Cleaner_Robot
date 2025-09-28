// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_stm/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "diffdrive_stm/ComDriver.h"

ComDriver com;

namespace diffdrive_stm
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  pid_parameter.max_count_per_loop = (int)hardware_interface::stod(info_.hardware_parameters["max_count_per_loop"]);
  pid_parameter.frequency = (int)hardware_interface::stod(info_.hardware_parameters["frequency"]);
  pid_parameter.p = hardware_interface::stod(info_.hardware_parameters["p"]);
  pid_parameter.i = hardware_interface::stod(info_.hardware_parameters["i"]);
  pid_parameter.d = hardware_interface::stod(info_.hardware_parameters["d"]);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  com.init();

  if(Com_Status_t::COM_OK != com.set_system_mode(com_system_state_t::SYSTEM_STATE_RESET))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  if(Com_Status_t::COM_OK != com.set_parameter(pid_parameter))
  {
    RCLCPP_INFO(get_logger(), "Fail to set parameter!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if(Com_Status_t::COM_OK != com.set_system_mode(com_system_state_t::SYSTEM_STATE_RUNNING))
  {
    RCLCPP_INFO(get_logger(), "Fail to set system mode!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  if(Com_Status_t::COM_OK != com.set_system_mode(com_system_state_t::SYSTEM_STATE_RESET))
  {
    RCLCPP_INFO(get_logger(), "Fail to set system mode to reset!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  com.close();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  com.read_velocity();
  // com.print_velocity();

  hw_velocities_[0] = convert_pid_loop_count_to_velocity(com.pid_count_per_loop_left);
  hw_velocities_[1] = convert_pid_loop_count_to_velocity(com.pid_count_per_loop_right);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_stm ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  com.write_velocity(
    convert_velocity_to_pid_loop_count(hw_commands_[0]), // left velocity
    convert_velocity_to_pid_loop_count(hw_commands_[1]) // right velocity
  );

  return hardware_interface::return_type::OK;
}

double DiffBotSystemHardware::convert_pid_loop_count_to_velocity(int8_t count_per_pid_loop)
{
    return (double)count_per_pid_loop / COUNT_PER_CYCLE * 3.1415 * 0.065 * pid_parameter.frequency;
}

int8_t DiffBotSystemHardware::convert_velocity_to_pid_loop_count(double velocity)
{
    return (int8_t)(velocity * COUNT_PER_CYCLE / 3.1415 / 0.065 / pid_parameter.frequency);
}

}  // namespace diffdrive_stm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_stm::DiffBotSystemHardware, hardware_interface::SystemInterface)
