import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
import numpy as np
import math

global x_coordinates_robot
global y_coordinates_robot

x_coordinates_robot = []
y_coordinates_robot = []

class RobotPlotting(Node):
    def __init__(self):
        super().__init__('robot_plotting')

        subscription_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
                )

        self.wheel_0_subscriber = self.create_subscription(
                Float64,
                '/actuators/esc/wheel_velocity_axis0',
                self.wheel_0_converter_cb,
                subscription_qos
                )

        self.wheel_1_subscriber = self.create_subscription(
                Float64,
                '/actuators/esc/wheel_velocity_axis1',
                self.wheel_1_converter_cb,
                subscription_qos
                )


        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.wheel_0_distance = 0.0
        self.wheel_1_distance = 0.0
        self.wheel_0_distance_list = []
        self.wheel_1_distance_list = []
        self.wheel_0_speed_list = []
        self.wheel_1_speed_list = []
        self.x_coordinates = []
        self.y_coordinates = []
        self.scan_x = []
        self.scan_y = []
        self.robot_distance_traveled = []
        self.change_in_orientation = []
        self.new_orientation = []
        self.wheel_radius = 0.135
        self.track_width = 0.67
        self.time_elapsed = 0.1


    def wheel_0_converter_cb(self, msg):
        wheel_0_speed = abs(msg.data / 4.294)
        self.wheel_0_speed_list.append(wheel_0_speed)


    def wheel_1_converter_cb(self, msg):
        wheel_1_speed = abs(msg.data / 4.294)
        self.wheel_1_speed_list.append(wheel_1_speed)
        self.robot_coordinates_calc()


    def robot_coordinates_calc(self):
        # Calculate distance traveled by each wheel
        for wheel_speed0, wheel_speed1 in zip(self.wheel_0_speed_list, self.wheel_1_speed_list):
            distance0 = wheel_speed0 * self.wheel_radius * self.time_elapsed
            distance1 = wheel_speed1 * self.wheel_radius * self.time_elapsed
            # Calculate the change in orientation of the robot
            delta_theta = (distance1 - distance0) / self.track_width

            # Calculate the new orientation of the robot
            self.robot_theta += delta_theta

            # Calculate the average distance traveled by the robot
            distance_traveled = (distance0 + distance1) / 2

            # Calculate the change in x and y coordinates of the robot
            delta_x = distance_traveled * math.cos(delta_theta)
            delta_y = distance_traveled * math.sin(delta_theta)

            # Calculate the new x and y coordinates of the robot
            self.robot_x += delta_x
            self.robot_y += delta_y

            x_coordinates_robot.append(self.robot_x)
            y_coordinates_robot.append(self.robot_y)


        # Plot the latest robot position
        self.robot_plotting()


    def robot_plotting(self):
        np_x_coordinates_robot = np.array(x_coordinates_robot)
        np_y_coordinates_robot = np.array(y_coordinates_robot)
        plt.plot(np_x_coordinates_robot, np_y_coordinates_robot, 'ro')
        plt.savefig('robot_trajectory.png')


def main(args=None):
    rclpy.init(args=args)
    robot_plotting = RobotPlotting()
    rclpy.spin(robot_plotting)
    robot_plotting.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
