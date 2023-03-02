import math
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from rclpy.node import Node
from haversine import haversine

class NavFix2Ccan(Node):
    def __init__(self, args):
        super().__init__("gps_fix")
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/gps', self.gps_callback, 10)
        self.dist_pub = self.create_publisher(Float32, "/gps/distance", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.prev = None
        self.dist = 0

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        self.gps_pub.publish(msg)
        if not math.isnan(latitude) and not math.isnan(longitude):
            delta = 0 if self.prev is None else haversine((self.prev[0], self.prev[1]), (latitude, longitude))
            self.dist = float(self.dist + delta)
            self.prev = [latitude, longitude]
            msg = Float32()
            msg.data = self.dist*1000
            self.dist_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navfix = NavFix2Ccan(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

