#! /usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class VelocityConverter(Node):
    def __init__(self):
        super().__init__('velocity_converter')

        subscription_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.wheel_0_subscriber = self.create_subscription(
            Float64,
            '/actuators/esc/wheel_velocity_axis0',
            self.wheel_0_motor_to_wheel_velocity_cb,
            subscription_qos
        )

        self.wheel_1_subscriber = self.create_subscription(
            Float64,
            '/actuators/esc/wheel_velocity_axis1',
            self.wheel_1_motor_to_wheel_velocity_cb,
            subscription_qos
        )

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/sensors/scan',
            self.laser_publisher_cb,
            subscription_qos
        )

        self.wheel_velocity_publisher = self.create_publisher(
                Twist,
                '/diffbot_base_controller/cmd_vel_unstamped',
                10
        )

        self.laser_publisher = self.create_publisher(
                LaserScan,
                '/hugo/scan',
                10
        )
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.wheel_0_speed_list = []
        self.wheel_1_speed_list = []
        
    def timer_cb(self):

        if len(self.wheel_0_speed_list) > 0 and len(self.wheel_1_speed_list) > 0:
            left_wheel_speed = self.wheel_0_speed_list.pop(0)
            right_wheel_speed = self.wheel_1_speed_list.pop(0)

            # Going straight
            if left_wheel_speed == right_wheel_speed:
                msg = Twist()
                msg.linear.x = left_wheel_speed
                self.wheel_velocity_publisher.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = min(left_wheel_speed, right_wheel_speed)
                msg.angular.z = left_wheel_speed - right_wheel_speed
                self.wheel_velocity_publisher.publish(msg)

    def laser_publisher_cb(self, msg):
        msg.header.frame_id = 'base_link'
        self.laser_publisher.publish(msg)

    def wheel_1_motor_to_wheel_velocity_cb(self, msg):
        wheel0_speed = -msg.data / 4.294
        self.wheel_0_speed_list.append(wheel0_speed)

    def wheel_0_motor_to_wheel_velocity_cb(self, msg):
        wheel1_speed = msg.data / 4.294
        self.wheel_1_speed_list.append(wheel1_speed)

def main(args=None):
    rclpy.init(args=args)
    velocity_converter = VelocityConverter()
    rclpy.spin(velocity_converter)
    velocity_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
