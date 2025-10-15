#!/usr/bin/env python3
"""
Simple test script to control the SHL-1 robot with grouped controllers.
This demonstrates how to send commands to all wheels at once.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class RobotControlTest(Node):
    def __init__(self):
        super().__init__('robot_control_test')
        
        # Publishers for grouped controllers
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
        
        self.get_logger().info('Robot Control Test Node Started')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /all_steer_position_controller/commands')
        self.get_logger().info('  - /all_drive_velocity_controller/commands')
        
    def set_steering(self, angles):
        """
        Set steering angles for all 6 wheels.
        
        Args:
            angles: List of 6 angles in radians [fl, fr, ml, mr, rl, rr]
        """
        if len(angles) != 6:
            self.get_logger().error('Must provide exactly 6 steering angles!')
            return
        
        msg = Float64MultiArray()
        msg.data = angles
        self.steer_pub.publish(msg)
        self.get_logger().info(f'Steering set to: {[f"{a:.2f}" for a in angles]}')
    
    def set_drive(self, velocities):
        """
        Set drive velocities for all 6 wheels.
        
        Args:
            velocities: List of 6 velocities in rad/s [fl, fr, ml, mr, rl, rr]
        """
        if len(velocities) != 6:
            self.get_logger().error('Must provide exactly 6 drive velocities!')
            return
        
        msg = Float64MultiArray()
        msg.data = velocities
        self.drive_pub.publish(msg)
        self.get_logger().info(f'Drive set to: {[f"{v:.2f}" for v in velocities]}')
    
    def stop(self):
        """Stop all wheels."""
        self.set_drive([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.get_logger().info('Robot stopped')
    
    def center_steering(self):
        """Center all steering joints."""
        self.set_steering([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.get_logger().info('Steering centered')


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlTest()
    
    try:
        # Give time for connections to establish
        time.sleep(1.0)
        
        # Test 1: Center steering
        node.get_logger().info('\n=== Test 1: Center Steering ===')
        node.center_steering()
        time.sleep(2.0)
        
        # Test 2: Turn all wheels left
        node.get_logger().info('\n=== Test 2: Turn All Wheels Left (30°) ===')
        angle = math.radians(30)
        node.set_steering([angle, angle, angle, angle, angle, angle])
        time.sleep(5.0)
        
        # Test 3: Turn all wheels right
        node.get_logger().info('\n=== Test 3: Turn All Wheels Right (-30°) ===')
        angle = math.radians(-30)
        node.set_steering([angle, angle, angle, angle, angle, angle])
        time.sleep(5.0)
        
        # Test 4: Center and drive forward
        node.get_logger().info('\n=== Test 4: Center Steering and Drive Forward ===')
        node.center_steering()
        time.sleep(2.0)
        node.set_drive([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        time.sleep(5.0)
        
        # Test 5: Stop
        node.get_logger().info('\n=== Test 5: Stop ===')
        node.stop()
        time.sleep(1.0)
        
        # Test 6: Ackermann-like steering (simplified)
        node.get_logger().info('\n=== Test 6: Ackermann Turn (Left) ===')
        # Front wheels turn more, rear wheels turn less
        fl = math.radians(30)
        fr = math.radians(25)
        ml = math.radians(15)
        mr = math.radians(12)
        rl = math.radians(5)
        rr = math.radians(4)
        node.set_steering([fl, fr, ml, mr, rl, rr])
        time.sleep(2.0)
        node.set_drive([1.5, 1.8, 1.6, 1.9, 1.7, 2.0])  # Outer wheels faster
        time.sleep(10.0)
        
        # Test 7: Stop and reset
        node.get_logger().info('\n=== Test 7: Stop and Reset ===')
        node.stop()
        time.sleep(1.0)
        node.center_steering()
        
        node.get_logger().info('\n=== All Tests Complete ===')
        
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()