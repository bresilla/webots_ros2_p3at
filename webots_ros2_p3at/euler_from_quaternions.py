import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import message_filters
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
     
        return roll_x, pitch_y, yaw_z # in radians

class EulerQu(Node):
    def __init__(self, args):
        super().__init__("quaterions_euler")
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/gps')
        self.pose_sub = self.create_subscription(Odometry, '/odometry/local', self.pose_callback, 10)

    def pose_callback(self, msg):
        qu = msg.pose.pose.orientation
        eu = euler_from_quaternion(qu.x, qu.y, qu.z, qu.w)
        print(eu)

def main(args=None):
    rclpy.init(args=args)
    eulerqu = EulerQu(args=args)

    rclpy.spin(eulerqu)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


