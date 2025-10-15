#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        
        # Declare parameters (with defaults from URDF)
        self.declare_parameter('wheel_radius', 0.0825)
        self.declare_parameter('wheelbase', 0.54)
        self.declare_parameter('track_width', 0.53)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        wheelbase = self.get_parameter('wheelbase').value
        track = self.get_parameter('track_width').value
        
        # Calculate wheel positions from geometry
        x_f = wheelbase / 2.0   # front
        x_m = 0.0                # middle
        x_r = -wheelbase / 2.0   # rear
        y_l = track / 2.0        # left
        y_r = -track / 2.0       # right
        
        # Wheel positions in order: FL, FR, ML, MR, RL, RR
        self.wheel_xy = [
            (x_f, y_l),  # front_left
            (x_f, y_r),  # front_right
            (x_m, y_l),  # middle_left
            (x_m, y_r),  # middle_right
            (x_r, y_l),  # rear_left
            (x_r, y_r),  # rear_right
        ]
        
        # Publishers for joint commands
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/all_steer_position_controller/commands',
            10
        )
        
        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            '/all_drive_velocity_controller/commands',
            10
        )
        
        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Swerve controller initialized')
        self.get_logger().info(f'Wheel positions: {self.wheel_xy}')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}')
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist command to individual wheel commands"""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        steer_cmd = []
        drive_cmd = []
        
        pi_2 = np.pi / 2.0
        
        for i, (xi, yi) in enumerate(self.wheel_xy):
            # Calculate linear velocity at wheel contact point
            vix = vx - wz * yi
            viy = vy + wz * xi
            
            # Convert linear velocity vector to joint commands
            steering_angle = np.arctan2(viy, vix)
            drive_speed = np.sqrt(vix**2 + viy**2)
            drive_joint_velocity = drive_speed / self.wheel_radius
            
            # Handle zero velocity case
            if abs(vix) < 1e-6 and abs(viy) < 1e-6:
                steering_angle = 0.0
                drive_joint_velocity = 0.0
            
            # Swerve optimization: keep steering in [-90, 90] degrees
            # by reversing drive direction if needed
            if steering_angle > pi_2:
                steering_angle -= np.pi
                drive_joint_velocity *= -1
            elif steering_angle < -pi_2:
                steering_angle += np.pi
                drive_joint_velocity *= -1
            
            steer_cmd.append(steering_angle)
            drive_cmd.append(drive_joint_velocity)
        
        # Publish commands
        steer_msg = Float64MultiArray()
        steer_msg.data = steer_cmd
        self.steer_pub.publish(steer_msg)
        
        drive_msg = Float64MultiArray()
        drive_msg.data = drive_cmd
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()