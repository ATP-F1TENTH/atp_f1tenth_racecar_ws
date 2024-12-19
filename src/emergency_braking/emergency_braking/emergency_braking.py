import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
import math

TOPIC_EMERGENCY_BRAKE = "/emergency_brake"
TOPIC_LASERSCAN = "/scan"
TOPIC_DRIVE = "/drive"
TOPIC_ODOMETRY = "/odom"

TTC_TRIGGER_S = 0.1
VIEW_ANGLE = 15.0   # TODO: Always view into the actual drive direction

class AEB(Node):
    _last_msg: LaserScan
    _current_speed: Float32
    _current_speed = 0.0
    _current_steering_angle_rad: Float32
    _current_steering_angle_rad = 0.0
    

    # _last_odom: Odometry
    ttc_arr = []
    def __init__(self):
        super().__init__('emergency_braking')
        print(f"AEB Start", flush=True)
        self.sub_laser = self.create_subscription(LaserScan, TOPIC_LASERSCAN, self.callback_new_laserscan, 10)
        self.sub_laser  # prevent unused variable warning
        self.sub_drive = self.create_subscription(AckermannDriveStamped, TOPIC_DRIVE, self.callback_new_drive, 10)
        self._last_msg = LaserScan()
        self.sub_odom = self.create_subscription(Odometry, TOPIC_ODOMETRY, self.callback_new_odometry, 10)
        self.sub_odom
        self._last_odom = Odometry()
        self.break_flag = False

        self.pub_brake = self.create_publisher(Bool, TOPIC_EMERGENCY_BRAKE, 10)
        msg_b = Bool()
        msg_b.data = False
        self.pub_brake.publish(msg_b)


    def callback_new_drive(self, msg: AckermannDriveStamped):
        #self._current_speed = msg.drive.speed
        self._current_steering_angle_rad = msg.drive.steering_angle
        
        
    def callback_new_odometry(self, msg:Odometry):
        self._current_speed = msg.twist.twist.linear.x


    def callback_new_laserscan(self, msg: LaserScan):
        center_idx = int(len(msg.ranges)/2.0)

        angle_offset_rad = self._current_steering_angle_rad

        start_idx = int ( max( (-math.radians(VIEW_ANGLE/2)+angle_offset_rad ) / msg.angle_increment + center_idx, 0 ) )
        stop_idx = int ( min( (+math.radians(VIEW_ANGLE/2)+angle_offset_rad)  / msg.angle_increment + center_idx, len(msg.ranges)-1 ) )
        ttc_center_idx = int((stop_idx-start_idx)/2)

        self.ttc_arr = []
        for idx in range(start_idx, stop_idx):
            if len(self._last_msg.ranges) == 0:
                print("last msg is empty. Break for loop", flush=True)
                self._last_msg = msg
                return

            rel_speed = self._current_speed # TODO: When odom is ready, last_speed should be the actual speed and not the set speed
            if rel_speed <= 0:
                self.ttc_arr.append(math.inf)
            else:
                ttc = msg.ranges[idx]/rel_speed
                self.ttc_arr.append(ttc)

        ttc_means = self.ttc_arr     
            
        if len(ttc_means) > ttc_center_idx:
            min_ttc = min(ttc_means)

            if min_ttc < TTC_TRIGGER_S :
                print("AEB Engaged", flush=True)
                msg_b = Bool()
                msg_b.data = True
                self.pub_brake.publish(msg_b)
                self.break_flag = True
                
        else:
            print(self.ttc_arr)
        self._last_msg = msg


def main(args=None):
    rclpy.init(args=args)

    aeb = AEB()

    rclpy.spin(aeb)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aeb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


