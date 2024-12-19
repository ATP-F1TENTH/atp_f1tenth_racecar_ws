# -------------------------------------------------------------------------------------
# Import section:
# -------------------------------------------------------------------------------------

import rclpy 
from rclpy.node import Node 

from sensor_msgs.msg import LaserScan 
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped, Quaternion


import numpy as np
import math

from copy import deepcopy

# from print_color import print

# -------------------------------------------------------------------------------------
# Topics definition:
# -------------------------------------------------------------------------------------

TOPIC_DRIVE = "/to_drive" 
TOPIC_LASERSCAN = "/scan"
TOPIC_DIRECTION = "/direction"

# -------------------------------------------------------------------------------------
# Global constants definition:
# -------------------------------------------------------------------------------------

MAX_ACCEPTED_DISTANCE = 5       # sets value to cap distance
SAFETYBUBBLE_SIZE = 1.5         # sets size (radius) of the safetybubble for closest object
FILTERWINDOW_SIZE = 10          # sets size of filterwindow for smoothingfilter

CURVETURE_VALUE = 1             # determines distance to border in turns

MAX_V= 3 + 1.5                        # determine maximal possible driving speed
MIN_V = 0.5 +0.6                    # determin minmal possible speed
FOV = 2.7                       # sets FOV
CONVERGENCE= 2 + 1.0                  
# -------------------------------------------------------------------------------------
# Create class for Helperfunctions:
# -------------------------------------------------------------------------------------

class HelperFunctions():

    # Filterfunction to smooth the recieved dist ranges from msg.ranges
    @staticmethod
    def apply_smoothing_filter(input_vector, smoothing_filter_size):
        # If input vector is smaller than twice the smoothing size, return it unchanged 
        if len(input_vector) < 2 * smoothing_filter_size: 
            return input_vector 

        # Convolution operation for smoothing the input vector 
        # np.ones creates an array of ones which acts as a smoothing filter 
        # The divisor normalizes the filter to maintain the scale of input_vector 
        smoothed_vector = np.convolve(input_vector, np.ones((2*smoothing_filter_size+1,)) / (2*smoothing_filter_size+1), mode='valid') 

        # Extend the smoothed array to the original length by padding the start and end with original values 
        return np.r_[np.full(smoothing_filter_size, input_vector[0]), smoothed_vector, np.full(smoothing_filter_size, input_vector[-1])]

    # find the largest gap/sequence of non-zero values in ranges
    @staticmethod
    def find_largest_nonzero_sequence(input_vector):
        max_length = 0 
        max_index = -1 
        current_length = 0 
        current_index = -1 

        for i, number in enumerate(input_vector): 

            if number > 0: 
                if current_index == -1: 
                    current_index = i 
                current_length += 1 

            else: 
                if current_length > max_length: 
                    max_length = current_length 
                    max_index = current_index 

                current_length = 0 
                current_index = -1 

        if current_length > max_length: 
            max_length = current_length 
            max_index = current_index 

        return max_length, max_index 
    
    
    # add SAFETYBUBBLE to the closeset point and substitute values within bubble with zero
    @staticmethod
    def find_points_in_bubble(input_vector):
        # Find the minimum distance in the vector 
        min_distance = np.min(input_vector) 

        # Set threshold to this minimum distance plus the car's radius 
        threshold_distance = min_distance + SAFETYBUBBLE_SIZE 

        # Return vector where all values less than or equal to the threshold are set to zero
        # print("Points in bubble around closest obstacle set to \"0\"", tag = "INFO", tag_color = "cyan", color = "white")
        return np.where(input_vector <= threshold_distance, 0, input_vector)

    
    # find best index in biggest gap:
    @staticmethod
    def find_best_index(input_vector, start_gap, max_gap_size):
        # currently "best index" defined as center of gap
        best_idx = (start_gap + max_gap_size) / 2
        
        print("best_index =", best_idx, flush=True) #, = "INFO", tag_color = "cyan", color = "white")
        return best_idx
    # get the longest distance in biggest gap
    @staticmethod
    def heighest_dist_gap(num_ranges, start_idx, gap_size):
        return np.max(num_ranges[start_idx : start_idx + gap_size])
    
    # determine the field of view of from the LIDAR sent message
    @staticmethod
    def adjust_field_of_view(input_vector, angle_increment):
        # Calculate the number of elements to delete from each side of the array
        truncated_angle = FOV // 2
        values_to_be_deleted = int(truncated_angle // angle_increment)

        # Calculate the start and end indices to keep in the array
        start_index = values_to_be_deleted
        end_index = len(input_vector) - values_to_be_deleted

        # Use slicing to efficiently create the final array
        final_array = input_vector[start_index:end_index]

        return final_array
    
    # calculate applied velocity by gap width distance of best index
    @staticmethod
    def calculate_dynamic_velocity(gap_length, max_distance, steering_angle):
        # returns calculated velocity in m/s
        first_velocity = (MAX_V+MIN_V)*math.exp(-CONVERGENCE*abs(steering_angle))+MIN_V
        #if max_distance <= 3:
            #return first_velocity * 0.5
        return first_velocity
    
    # Quarternion Calculation:
    @staticmethod
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    
    
# -------------------------------------------------------------------------------------
# Create class for GapFollowNode:
# -------------------------------------------------------------------------------------

class GapFollowNode(Node):

    def __init__(self): 
        super().__init__('gap_follow') 
        
        # Subscriber definition and initialization to TOPIC_LASERSCAN
        self.sub_laser = self.create_subscription(LaserScan, TOPIC_LASERSCAN, self.callback_new_laserscan, 10) 
        
        # Publischer definition and initialization to TOPIC LASERSCAN
        self.pub_drive = self.create_publisher(AckermannDriveStamped, TOPIC_DRIVE, 10)

        # Publischer definition and initialization to TOPIC DIRECTION
        self.pub_direction = self.create_publisher(PoseStamped, TOPIC_DIRECTION, 10)

        # print(f"Waiting for LaserScan messages on topic {TOPIC_LASERSCAN}", tag = "INFO", tag_color = "cyan", color = "white") 
    
    def callback_new_laserscan(self, msg : LaserScan):
        # determine fov_size
        fov_ranges = HelperFunctions.adjust_field_of_view(msg.ranges, msg.angle_increment)

        # process and filter ranges
        filtered_ranges = self.preprocess_lidar(fov_ranges)

        # find closest obstacle and raplace values within safetybubble by 0
        filtered_ranges = HelperFunctions.find_points_in_bubble(filtered_ranges)
        
        # find biggest gap
        max_gap_size, idx_gap_start = HelperFunctions.find_largest_nonzero_sequence(filtered_ranges)
        print("max_gap_size = ", max_gap_size, "inx_gap_start = ", idx_gap_start, flush=True)#, tag = "INFO", tag_color = "cyan", color = "white")
        
        if max_gap_size > 0:
            # best_idx = HelperFunctions.find_best_index(filtered_ranges, idx_gap_start, max_gap_size)

            max_dist = HelperFunctions.heighest_dist_gap(filtered_ranges, idx_gap_start, max_gap_size)
            steering_angle, velocity = self.configurate_driveparameters(max_dist, idx_gap_start, max_gap_size, fov_ranges, msg.angle_increment)
            print(f"Velocity = {velocity};\t Steering Angle = {steering_angle}", flush = True)#, tag = "INFO", tag_color = "cyan", color = "white")
            self.publish_drive_command(steering_angle, velocity)
            #self.publisch_direction_message(steering_angle)
        
        return None

    def preprocess_lidar(self, ranges): 
        # Replace NaN with zero, and cap values by ACCEPTED_DISTANCE 
        filtered_ranges_1 = np.where(np.isnan(ranges), 0, np.minimum(ranges, MAX_ACCEPTED_DISTANCE))
        
        # Replace Inf with linear interpolated value
        inf_indices = [i for i, x in enumerate(filtered_ranges_1) if np.isinf(x)]
        filtered_ranges_2 = deepcopy(filtered_ranges_1)

        for i in inf_indices:
            # find the neares non-infinite neigbors
            left_indx = i - 1
            right_indx = i + 1
            while left_indx in inf_indices:
                left_indx -= 1
            while right_indx in inf_indices:
                right_indx += 1

            # linear interpolation:
            left_val = filtered_ranges_1[left_indx]
            right_val = filtered_ranges_1[right_indx]
            interval_length = right_indx - left_indx
            interpolated_value = left_val + ((i-left_indx) / interval_length) * (right_val - left_val)

            filtered_ranges_2[i] = interpolated_value

        # Apply a smoothing filter to the lidar range measurements 
        return HelperFunctions.apply_smoothing_filter(filtered_ranges_2, FILTERWINDOW_SIZE)

    # calculate given velocity and steering angle for drive message
    def configurate_driveparameters(self, max_distance, start_idx, gap_size, num_ranges, angle_increment): 
        # STEERING ANGLE:
        # define center index of fov
        center_index = len(num_ranges) // 2

        # get the "edge"-ranges of the biggest gap
        d1 = num_ranges[start_idx]
        d2 = num_ranges[start_idx + gap_size-1]

        # calculate the necessary angles
        angle_1 = (start_idx - center_index) * angle_increment
        angle_2 = (start_idx + gap_size - center_index) * angle_increment

        # Calculate the numerator and denominator for the acos argument
        numerator = d1 + d2 * math.cos(angle_1 + angle_2)
        denominator = math.sqrt(d1 ** 2 + d2 ** 2 + 2 * d1 * d2 * math.cos(angle_1+ angle_2))

        # Compute the new steering angle
        #new_angle = math.acos(numerator / denominator) - angle_difference_1
        help_angle = (angle_1 + angle_2) / 2
        d_min = np.min(num_ranges)
        constant= CURVETURE_VALUE/d_min

        steering_angle= (constant*help_angle)/(constant+1)

        # VELOCITY:
        # calculate the gep lenght in m by using "Cosine Rule"
        gap_length = math.sqrt(d1 ** 2 + d2 ** 2 - 2 * d1 * d2 * math.cos(abs(angle_1) + abs(angle_2)))
        velocity = HelperFunctions.calculate_dynamic_velocity(gap_length, max_distance, steering_angle)

        return steering_angle, velocity

    # Publish drive command to /drive topic
    def publish_drive_command(self, steering_angle, velocity):
        # configurate drive_msg to publish
        drive_msg = AckermannDriveStamped()

        drive_msg.header.stamp = self.get_clock().now().to_msg()

        drive_msg.header.frame_id = "laser"

        drive_msg.drive.speed = velocity

        drive_msg.drive.steering_angle = steering_angle

        self.pub_drive.publish(drive_msg)

        return None
    
    # Publish direction parameters to /direction topic
    def publisch_direction_message(self, steering_angle):
        # configurate direction_msg
        aimed_angle = steering_angle

        direction_msg = PoseStamped()

        direction_msg.header.stamp = self.get_clock().now().to_msg()

        direction_msg.header.frame_id = "ego_racecar/laser_model"

        direction_msg.pose.position = Point(x=0.0,y=0.0,z=0.0)

        q = HelperFunctions.quaternion_from_euler(0,0, aimed_angle)

        direction_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.pub_direction.publish(direction_msg)

        return None
        




def main(args=None): 

    rclpy.init(args=args) 

    node = GapFollowNode() 

    rclpy.spin(node) 

    node.destroy_node() 

    rclpy.shutdown() 

 
 

if __name__ == '__main__': 

    main() 
