#include "rse_shl1_swerve_controller/swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rse_shl1_swerve_controller
{

SwerveController::SwerveController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn SwerveController::on_init()
{
  try
  {
    // Declare parameters with defaults matching your robot
    auto_declare<double>("wheel_radius", 0.0825);
    auto_declare<double>("wheelbase", 0.54);
    auto_declare<double>("track_width", 0.53);
    
    auto_declare<std::vector<std::string>>("steer_joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("drive_joints", std::vector<std::string>());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  wheelbase_ = get_node()->get_parameter("wheelbase").as_double();
  track_width_ = get_node()->get_parameter("track_width").as_double();
  
  steer_joint_names_ = get_node()->get_parameter("steer_joints").as_string_array();
  drive_joint_names_ = get_node()->get_parameter("drive_joints").as_string_array();

  if (steer_joint_names_.size() != 6 || drive_joint_names_.size() != 6)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 6 steer joints and 6 drive joints, got %zu and %zu",
      steer_joint_names_.size(), drive_joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Calculate wheel positions
  double x_f = wheelbase_ / 2.0;
  double x_m = 0.0;
  double x_r = -wheelbase_ / 2.0;
  double y_l = track_width_ / 2.0;
  double y_r = -track_width_ / 2.0;

  wheel_positions_ = {
    {x_f, y_l},  // front_left
    {x_f, y_r},  // front_right
    {x_m, y_l},  // middle_left
    {x_m, y_r},  // middle_right
    {x_r, y_l},  // rear_left
    {x_r, y_r}   // rear_right
  };

  // Create cmd_vel subscriber
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
      cmd_vel_callback(msg);
    });

  RCLCPP_INFO(get_node()->get_logger(), "Swerve controller configured successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SwerveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Steering joints - position interface
  for (const auto & joint_name : steer_joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  // Drive joints - velocity interface
  for (const auto & joint_name : drive_joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::InterfaceConfiguration
SwerveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // We need state interfaces for feedback
  for (const auto & joint_name : steer_joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  for (const auto & joint_name : drive_joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

controller_interface::CallbackReturn SwerveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Assign command interfaces
  steer_command_interfaces_.clear();
  drive_command_interfaces_.clear();

  for (const auto & joint_name : steer_joint_names_)
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint_name](const auto & interface) {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });

    if (it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected position command interface for '%s' not found",
        joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    steer_command_interfaces_.emplace_back(*it);
  }

  for (const auto & joint_name : drive_joint_names_)
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint_name](const auto & interface) {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });

    if (it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected velocity command interface for '%s' not found",
        joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    drive_command_interfaces_.emplace_back(*it);
  }

  // Initialize command buffer with zeros
  auto initial_cmd = std::make_shared<geometry_msgs::msg::Twist>();
  rt_command_ptr_.writeFromNonRT(initial_cmd);

  RCLCPP_INFO(get_node()->get_logger(), "Swerve controller activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all wheels
  for (auto & interface : steer_command_interfaces_)
  {
    interface.get().set_value(0.0);
  }
  for (auto & interface : drive_command_interfaces_)
  {
    interface.get().set_value(0.0);
  }

  steer_command_interfaces_.clear();
  drive_command_interfaces_.clear();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Get latest command
  auto cmd_vel = rt_command_ptr_.readFromRT();
  if (!cmd_vel || !(*cmd_vel))
  {
    return controller_interface::return_type::OK;
  }

  double vx = (*cmd_vel)->linear.x;
  double vy = (*cmd_vel)->linear.y;
  double wz = (*cmd_vel)->angular.z;

  // Compute swerve kinematics
  std::vector<double> steer_cmds(6);
  std::vector<double> drive_cmds(6);
  compute_swerve_kinematics(vx, vy, wz, steer_cmds, drive_cmds);

  // Send commands to hardware
  for (size_t i = 0; i < 6; ++i)
  {
    steer_command_interfaces_[i].get().set_value(steer_cmds[i]);
    drive_command_interfaces_[i].get().set_value(drive_cmds[i]);
  }

  return controller_interface::return_type::OK;
}

void SwerveController::cmd_vel_callback(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  rt_command_ptr_.writeFromNonRT(msg);
}

void SwerveController::compute_swerve_kinematics(
  double vx, double vy, double wz,
  std::vector<double> & steer_cmds,
  std::vector<double> & drive_cmds)
{
  const double pi_2 = M_PI / 2.0;

  for (size_t i = 0; i < 6; ++i)
  {
    double xi = wheel_positions_[i].x;
    double yi = wheel_positions_[i].y;

    // Calculate linear velocity at wheel contact point
    double vix = vx - wz * yi;
    double viy = vy + wz * xi;

    // Convert to steering angle and drive velocity
    double steering_angle = std::atan2(viy, vix);
    double drive_speed = std::sqrt(vix * vix + viy * viy);
    double drive_velocity = drive_speed / wheel_radius_;

    // Handle zero velocity case
    if (std::abs(vix) < 1e-6 && std::abs(viy) < 1e-6)
    {
      steering_angle = 0.0;
      drive_velocity = 0.0;
    }

    // Swerve optimization: keep steering in [-90, 90] degrees
    if (steering_angle > pi_2)
    {
      steering_angle -= M_PI;
      drive_velocity *= -1.0;
    }
    else if (steering_angle < -pi_2)
    {
      steering_angle += M_PI;
      drive_velocity *= -1.0;
    }

    steer_cmds[i] = steering_angle;
    drive_cmds[i] = drive_velocity;
  }
}

}  // namespace rse_shl1_swerve_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rse_shl1_swerve_controller::SwerveController,
  controller_interface::ControllerInterface)