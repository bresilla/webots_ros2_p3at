import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import message_filters
import threading
import numpy as np


class GoToPosition(Node):
    def __init__(self, x, y):
        super().__init__('go_to_position')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub1_ = self.create_subscription(Odometry, '/odometry/global', self.imu_callback, 10)
        self.sub2_ = self.create_subscription(Odometry, '/odometry/gps', self.gps_callback, 10)
        
        # self.gps_filtered = message_filters.Subscriber(self, Odometry, '/odometry/gps')
        # self.imu_data = message_filters.Subscriber(self, Odometry, '/odometry/global')
        # self.pose_sub = message_filters.ApproximateTimeSynchronizer([self.gps_filtered, self.imu_data], 10, slop=10)
        # self.pose_sub.registerCallback(self.pose_callback)

        self.thread = None
        self.current_pose_ = Point()
        self.target_pose_ = Point()
        self.target_pose_.x = x
        self.target_pose_.y = y
        self.current_orientation_ = Quaternion()
        self.factor = [10, 100]

    def gps_callback(self, msg):
        self.current_pose_ = msg.pose.pose.position

    def imu_callback(self, msg):
        self.current_orientation_ = msg.pose.pose.orientation

    def pose_callback(self, gps_odom, imu):
        self.current_pose_ = gps_odom.pose.pose.position
        self.current_orientation_ = imu.pose.pose.orientation

    def go(self):
        distance = math.sqrt((self.target_pose_.x - self.current_pose_.x) ** 2 + (self.target_pose_.y - self.current_pose_.y) ** 2)
        velocity = 0.1 * distance if distance < 10 else 0.5
        preheading = math.atan2(self.target_pose_.y - self.current_pose_.y, self.target_pose_.x - self.current_pose_.x)
        # print("PREHEADING: ", preheading)
        orientation = yaw = math.atan2(2 * (self.current_orientation_.w * self.current_orientation_.z + self.current_orientation_.x * self.current_orientation_.y), 1 - 2 * (self.current_orientation_.y**2 + self.current_orientation_.z**2))
        # print("ORIENTATION: ", orientation)
        heading = preheading - orientation
        # print("HEADING: ", heading)
        heading_corrected = np.arctan2(np.sin(heading), np.cos(heading))
        # print("HEADING_CORRECTED: ", heading_corrected)
        angular = heading_corrected
        print("")

        # heading = np.arctan2(np.sin(heading), np.cos(heading))
        return distance, velocity, angular

    def move_to_position(self):
        twist = Twist()
        while True:
            distance, twist.linear.x, twist.angular.z  = self.go()
            self.publisher_.publish(twist)
            print(twist)
            if distance < 1: 
                self.cleanup()
                break

    def cleanup(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


def main(args=None):
    if len(sys.argv) < 3:
        print('Usage: ros2 run <package> <node> x y')
        return

    x, y = float(sys.argv[1]), float(sys.argv[2])

    rclpy.init(args=args)
    goto = GoToPosition(x, y)

    if goto.thread is None or not goto.thread.is_alive():
        goto.thread = threading.Thread(target=goto.move_to_position)
        goto.thread.start()

    rclpy.spin(goto)

    goto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()