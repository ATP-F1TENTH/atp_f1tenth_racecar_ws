import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math
# Show case for import user defined class
from TestKlasse import TestClass as TC

TOPIC_DRIVE = "/drive"
TOPIC_LASERSCAN = "/scan"
TOPIC_ODOMETRY = "/ego_racecar/odom"

class MPCC(Node):
    def __init__(self):
        super().__init__('mpcc')
        self.sub_laser = self.create_subscription(LaserScan, TOPIC_LASERSCAN, self.callback_new_laserscan, 10)
        self.sub_laser  # prevent unused variable warning

        print(f"waiting for LaserScan messages on topic {TOPIC_LASERSCAN}")

    def callback_new_laserscan(self, msg: LaserScan):
        #we'll now output the 3 readings for forwards (positive x-axis), left (negative y-axis), and right (positive y-axis) from the scan
        #laserscan msgs are layout counterclockwise around the z-axis (pointing to the ceiling). the ray in the center of the list points forwards.

        center_idx = int(len(msg.ranges)/2.0)

        front_raylength_m = msg.ranges[center_idx]

        left_raylength_m  = msg.ranges[ int ( min( +math.radians(90.0)  / msg.angle_increment + center_idx, len(msg.ranges)-1 ) ) ]
        right_raylength_m = msg.ranges[ int ( max( -math.radians(90.0)  / msg.angle_increment + center_idx, 0 ) ) ]

        print(f"front {front_raylength_m}\tleft {left_raylength_m}\tright {right_raylength_m}", flush=True)


def main(args=None):
    rclpy.init(args=args)

    mpcc = MPCC()
    rclpy.spin(mpcc)

    # Destroy the node explicitly
    mpcc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()