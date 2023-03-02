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


def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z # in radians

def gotoGoal(goal, pose, orientation, precision=0.1, velocity=0.5, stop_loop=False):
    # calculate heading to goal
    preheading = np.arctan2((goal[1]-pose[1]), (goal[0]-pose[0]))
    heading = preheading - orientation
    heading_corrected = np.arctan2(np.sin(heading), np.cos(heading))

	# if (angDel <= -M_PI)
	# 	return angDel + M_PI * 2;
	# if (angDel > M_PI)
	# 	return angDel - M_PI * 2;
	# return angDel;



    heading_corrected = heading_corrected/np.pi
    if heading_corrected > 0.20:
        heading_corrected = 0.20
    if heading_corrected < -0.20:
        heading_corrected = -0.20
    #calculate distance to goal 
    distance = ((pose[0] - goal[0])**2 + (pose[1] - goal[1])**2)**0.5
    return float(velocity), heading_corrected, distance

class GoToPosition(Node):
    def __init__(self, x, y):
        super().__init__('go_to_position')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/odometry/local', self.odometry_callback, 10)
        
        self.gps_filtered = message_filters.Subscriber(self, Odometry, '/odometry/gps')
        self.imu_data = message_filters.Subscriber(self, Odometry, '/odometry/global')
        self.pose_sub = message_filters.ApproximateTimeSynchronizer([self.gps_filtered, self.imu_data], 10, slop=10)
        self.pose_sub.registerCallback(self.pose_callback)


        self.thread = None
        self.current_pose_ = Point()
        self.target_pose_ = Point()
        self.target_pose_.x = x
        self.target_pose_.y = y
        self.current_orientation_ = Quaternion()
        self.factor = [10, 100]

    def odometry_callback(self, msg):
        pass
        # self.current_pose_ = msg.pose.pose.position
        # self.current_orientation_ = msg.pose.pose.orientation

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
