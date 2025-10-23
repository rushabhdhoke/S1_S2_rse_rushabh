#ifndef RSE_SHL1_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
#define RSE_SHL1_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace rse_shl1_swerve_controller
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  SwerveController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Parameters
  double wheel_radius_;
  double wheelbase_;
  double track_width_;
  
  std::vector<std::string> steer_joint_names_;
  std::vector<std::string> drive_joint_names_;

  // Wheel positions (x, y) for each wheel
  struct WheelPosition {
    double x;
    double y;
  };
  std::vector<WheelPosition> wheel_positions_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_command_ptr_;

  // Command interfaces (references to hardware)
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    steer_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    drive_command_interfaces_;

private:
  void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  
  void compute_swerve_kinematics(
    double vx, double vy, double wz,
    std::vector<double> & steer_cmds,
    std::vector<double> & drive_cmds);
};

}  // namespace rse_shl1_swerve_controller

#endif  // RSE_SHL1_SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_