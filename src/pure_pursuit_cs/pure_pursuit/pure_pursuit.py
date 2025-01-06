import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, ColorRGBA
import csv
import os
import math
from transforms3d.euler import quat2euler
from tf2_ros import Buffer, TransformListener, TransformStamped
import numpy as np
import tf2_geometry_msgs
from transforms3d import quaternions




#======================================================================================
# Create class for Helperfunctions:
#======================================================================================

class Helperfunctions():
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
    
    # signum function
    @staticmethod
    def sgn (num):
        if num >= 0:
            return 1
        else:
            return -1
    
    # calculates the Eucledian distance between two points  
    @staticmethod   
    def calculate_distance(position1, position2):
    # position1 [array] coordinates for first point
    # position2 [waypoint] coordinates for second poitn
    # returns: Eucledian Distance
        return math.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2)
    
    @staticmethod
    def get_yaw_from_orientation(qw, qx, qy, qz):
        """
        Converts quaternion orientation to yaw angle.
        
        :param qw: Quaternion w component.
        :param qx: Quaternion x component.
        :param qy: Quaternion y component.
        :param qz: Quaternion z component.
        :return: Yaw angle.
        """
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def calculate_distance(point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    @staticmethod
    def calculate_dot_product(vector1, vector2):
        return vector1[0] * vector2[0] + vector1[1] * vector2[1]
    @staticmethod
    def calculate_vector(point1, point2):
        return [point2[0] - point1[0], point2[1] - point1[1]]
    @staticmethod
    def calculate_vector_magnitude(vector):
        return math.sqrt(vector[0] ** 2 + vector[1] ** 2)

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('race_line_csv', 'path/to/raceline.csv')
        self.declare_parameter('raceline_topic', '/race_line')
        self.declare_parameter('raceline_frame', 'map')
        self.declare_parameter('debug_marker_topic', '/pure_pursuit_debug')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', '/odom')
        self.declare_parameter('direction_topic', '/direction')
        self.declare_parameter('raceLine_pos_topic', '/race_position')
        

        self.race_line_csv = self.get_parameter('race_line_csv').get_parameter_value().string_value
        self.raceline_topic = self.get_parameter('raceline_topic').get_parameter_value().string_value
        self.raceline_frame = self.get_parameter('raceline_frame').get_parameter_value().string_value
        self.debug_topic = self.get_parameter('debug_marker_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.direction_topic = self.get_parameter('direction_topic').get_parameter_value().string_value
        self.race_line_pos_topic = self.get_parameter('raceLine_pos_topic').get_parameter_value().string_value

        # Transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lookahead_distance = 0.2
        self.look_max=1.6
        self.look_min=0.6
        self.speed_idx_diff = 4
        self.convergence=2

        self.wheelbase = 0.32
        self.distance_index= 0.2
        self.race_line = self.load_race_line(self.race_line_csv)
        self.raceline_marker_pub = self.create_publisher(MarkerArray, self.raceline_topic, 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, self.debug_topic, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
            # publisher for pose, to visualize direction/orientation in rviz
        self.direction_publischer = self.create_publisher(PoseStamped,self.direction_topic,10)
            # publisher for race position - for lap timer -> 0..100% of track
        self.race_position_pub = self.create_publisher(Int16, self.race_line_pos_topic, 10)

        self.pose_pub = self.create_publisher(PoseStamped, 'pose_in_map', 10)
        
        #publisher for debug values
        self.debug_pub = self.create_publisher(Float32, 'debug', 10)
        
        self.create_timer(0.1, self.publish_race_line)
        
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.current_pose : PoseStamped = PoseStamped()
        self.current_speed = 0.0
        self.current_pose_odom : PoseStamped = PoseStamped()
        self.target_index = 0
        self.lookahead_point = None
        self.sol1 = None
        self.sol2 = None
        self.minX = None
        self.minY = None
        self.maxX = None
        self.maxY = None
        self.speed_idx = None
        self.cross_track_error = None

    def load_race_line(self, file_path):
        """ Function to load trajectory from csv file. """

        delimiter = '\t'    # ','
        raceline = []

        with open(file_path, 'r') as file:
            reader = csv.reader(file, delimiter=delimiter)

            for row in reader:
                try:
                    x = float(row[0])
                    y = float(row[1])
                    v = float(row[2])
                    raceline.append((x, y, v))

                except ValueError as e:
                    self.get_logger().info(f"Skipped header row: {row}")
                    continue

        return raceline
    
    def _get_pointarray(self, points):
        pt_arr = []
        for p in points:
            pt_arr.append(Point(x=p[0], y=p[1]))
        return pt_arr
    
    def _get_colorarray(self, speeds):
        col_arr = []
        max_v = max(speeds) if speeds else 1.0
        
        for v in speeds:
            col_arr.append(ColorRGBA(r=1.0 - v / max_v,
                                     g=v / max_v,
                                     b=0.0,
                                     a=1.0))
        return col_arr
    
    def publish_race_line(self):
        # publish raceline as markers
        raceline_marker_array = MarkerArray()
        debug_marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.raceline_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "race_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP 
        marker.action = Marker.ADD
        marker.points = self._get_pointarray(self.race_line)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.colors = self._get_colorarray([point[2] for point in self.race_line])

        raceline_marker_array.markers.append(marker)
        
        marker = Marker()
        marker.header.frame_id = self.raceline_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "start"
        marker.id = 555
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.race_line[0][0]
        marker.pose.position.y = self.race_line[0][1]
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.a = 1.0
        max_v = max([point[2] for point in self.race_line]) if self.race_line else 1.0
        marker.color.r = 1.0 
        marker.color.g = 0.0
        marker.color.b = 0.0
        raceline_marker_array.markers.append(marker)


        # ppublish the current lookaheadpoint if present
        if self.lookahead_point != None:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = self.raceline_frame
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "lookahead_point"
            lookahead_marker.id = len(self.race_line)
            lookahead_marker.type = Marker.CYLINDER
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = self.lookahead_point[0]
            lookahead_marker.pose.position.y = self.lookahead_point[1]
            lookahead_marker.pose.position.z = 0.0
            lookahead_marker.scale.x = 0.2
            lookahead_marker.scale.y = 0.2
            lookahead_marker.scale.z = 0.2
            lookahead_marker.color.a = 1.0
            lookahead_marker.color.r = 0.0
            lookahead_marker.color.g = 0.0
            lookahead_marker.color.b = 1.0
            debug_marker_array.markers.append(lookahead_marker)
            
         # ppublish the current lookaheadpoint if present
        if self.sol1 != None:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = self.raceline_frame
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "sol1"
            lookahead_marker.id = len(self.race_line)
            lookahead_marker.type = Marker.SPHERE
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = self.sol1[0]
            lookahead_marker.pose.position.y = self.sol1[1]
            lookahead_marker.pose.position.z = 0.0
            lookahead_marker.scale.x = 0.2
            lookahead_marker.scale.y = 0.2
            lookahead_marker.scale.z = 0.2
            lookahead_marker.color.a = 1.0
            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 1.0
            lookahead_marker.color.b = 0.0
            debug_marker_array.markers.append(lookahead_marker)
            
        # ppublish the current lookaheadpoint if present
        if self.sol2 != None:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = self.raceline_frame
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "sol2"
            lookahead_marker.id = len(self.race_line)
            lookahead_marker.type = Marker.SPHERE
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = self.sol2[0]
            lookahead_marker.pose.position.y = self.sol2[1]
            lookahead_marker.pose.position.z = 0.0
            lookahead_marker.scale.x = 0.2
            lookahead_marker.scale.y = 0.2
            lookahead_marker.scale.z = 0.2
            lookahead_marker.color.a = 1.0
            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 0.0
            lookahead_marker.color.b = 0.0
            debug_marker_array.markers.append(lookahead_marker)
            
            
           # ppublish the current lookaheadpoint if present
        if self.maxX != None:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = self.raceline_frame
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "max"
            lookahead_marker.id = 666
            lookahead_marker.type = Marker.CUBE
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = self.maxX
            lookahead_marker.pose.position.y = self.maxY
            lookahead_marker.pose.position.z = 0.0
            lookahead_marker.scale.x = 0.2
            lookahead_marker.scale.y = 0.2
            lookahead_marker.scale.z = 0.2
            lookahead_marker.color.a = 1.0
            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 0.0
            lookahead_marker.color.b = 0.0
            debug_marker_array.markers.append(lookahead_marker)
        
           # ppublish the current lookaheadpoint if present
        if self.speed_idx != None:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = self.raceline_frame
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "min"
            lookahead_marker.id = 999
            lookahead_marker.type = Marker.CYLINDER
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = self.race_line[self.speed_idx][0]
            lookahead_marker.pose.position.y = self.race_line[self.speed_idx][1]
            lookahead_marker.pose.position.z = 0.0
            lookahead_marker.scale.x = 0.2
            lookahead_marker.scale.y = 0.2
            lookahead_marker.scale.z = 0.5
            lookahead_marker.color.a = 1.0
            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 0.0
            lookahead_marker.color.b = 1.0
            debug_marker_array.markers.append(lookahead_marker)
            
        if self.target_index != None:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = self.raceline_frame
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "target"
            lookahead_marker.id = 777
            lookahead_marker.type = Marker.CUBE
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = self.race_line[self.target_index][0]
            lookahead_marker.pose.position.y = self.race_line[self.target_index][1]
            lookahead_marker.pose.position.z = 0.0
            lookahead_marker.scale.x = 0.2
            lookahead_marker.scale.y = 0.2
            lookahead_marker.scale.z = 0.2
            lookahead_marker.color.a = 1.0
            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 0.0
            lookahead_marker.color.b = 1.0
            debug_marker_array.markers.append(lookahead_marker)
            
        self.raceline_marker_pub.publish(raceline_marker_array)
        self.debug_marker_pub.publish(debug_marker_array)
        
        
    def calc_lookahead_based_on_vel(self, waypoints, current_x, current_y):
        current_position = [current_x, current_y]
        distances = [Helperfunctions.calculate_distance(current_position, [waypoints[i][0], waypoints[i][1]]) for i in range(len(waypoints))]
        min_distance = min(distances)
        idx = distances.index(min_distance)
        vel = waypoints[idx][2]
        self.lookahead_distance = (self.look_max / (1+ math.exp(-6*(vel-3))))+self.look_min
        
        
    def calc_lookahead_based_on_crosserror(self, waypoints, current_x, current_y):
        current_position = [current_x, current_y]
        distances = [Helperfunctions.calculate_distance(current_position, [waypoints[i][0], waypoints[i][1]]) for i in range(len(waypoints))]
        min_distance = min(distances)
        idx = distances.index(min_distance)
        closest_point = waypoints[idx]

        if idx < len(waypoints) - 1:
            next_point = waypoints[idx + 1]
        else:
            next_point = waypoints[idx - 1]

        vector_path = Helperfunctions.calculate_vector([closest_point[0], closest_point[1]], [next_point[0], next_point[1]])
        vector_position = Helperfunctions.calculate_vector([closest_point[0], closest_point[1]], current_position)

        vector_path_magnitude = Helperfunctions.calculate_vector_magnitude(vector_path)
        projection_length = Helperfunctions.calculate_dot_product(vector_position, vector_path) / (vector_path_magnitude ** 2)
        projection = [projection_length * vector_path[0], projection_length * vector_path[1]]

        cross_track_vector = [vector_position[0] - projection[0], vector_position[1] - projection[1]]
        self.cross_track_error = Helperfunctions.calculate_vector_magnitude(cross_track_vector)

        self.lookahead_distance = (self.look_max - self.look_min) * math.exp(-self.convergence * abs(self.cross_track_error)) + self.look_min
        #print(self.lookahead_distance, flush=True)
        
    def tf_pose_odom_into_raceline_frame(self):
        try:
            # get TF
            raceline_to_odom = self.tf_buffer.lookup_transform(self.raceline_frame, self.odom_frame, rclpy.time.Time())

            # Compute the inverse transformation
            odom_to_raceline = TransformStamped()
            odom_to_raceline.header.stamp = raceline_to_odom.header.stamp
            odom_to_raceline.header.frame_id = raceline_to_odom.child_frame_id
            odom_to_raceline.child_frame_id = raceline_to_odom.header.frame_id

            # Compute the inverse transformation
            trans = raceline_to_odom.transform.translation
            rot = raceline_to_odom.transform.rotation
            translation = np.array([trans.x, trans.y, trans.z])
            rotation = np.array([rot.w, rot.x, rot.y, rot.z])

            rot_matrix = quaternions.quat2mat(rotation)     # get rot matrix
            inv_rot_matrix = rot_matrix.T                   # get inverse rot matrix

            inv_translation = -np.dot(inv_rot_matrix, translation)  #inverse translation
            inv_rotation = quaternions.mat2quat(inv_rot_matrix)     #inverse rotation

            odom_to_raceline.transform.translation.x = inv_translation[0]
            odom_to_raceline.transform.translation.y = inv_translation[1]
            odom_to_raceline.transform.translation.z = inv_translation[2]
            odom_to_raceline.transform.rotation.x = inv_rotation[1]
            odom_to_raceline.transform.rotation.y = inv_rotation[2]
            odom_to_raceline.transform.rotation.z = inv_rotation[3]
            odom_to_raceline.transform.rotation.w = inv_rotation[0]

            # TF pose (within odom frame) into a pose (within the global reference frame: raceline frame)
            self.current_pose = tf2_geometry_msgs.do_transform_pose_stamped(self.current_pose_odom, raceline_to_odom)

            self.pose_pub.publish(self.current_pose)

        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transform: {e}")
            # when tf failes, do not update pose -> hope that next tf is valid
            return

    def odom_callback(self, msg : Odometry):
        self.current_pose_odom.pose = msg.pose.pose
        self.current_speed = msg.twist.twist.linear.x
        self.current_pose_odom.header.stamp = msg.header.stamp 
        self.current_pose_odom.header.frame_id = msg.header.frame_id 
        self.tf_pose_odom_into_raceline_frame()
        self.follow_race_line()

    def get_lookahead_point(self):
        # Find the point that is the nearest the cars position, but is at least the lookahead_distance away
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        # Init the min_distance found
        min_distance = float('inf')
        lookahead_point = None

        for i in range(len(self.race_line)):
            x, y, v = self.race_line[i]
            distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
            
            # Check if the point is further than lookahead_distance from the current position
            if distance >= self.lookahead_distance and distance < min_distance:
                # Ensure the point is in front of the vehicle
                angle_to_point = math.atan2(y - current_y, x - current_x)
                current_orientation = self.current_pose.pose.orientation
                current_yaw = quat2euler([current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z])[2]
                angle_diff = angle_to_point - current_yaw

                if abs(angle_diff) < math.pi / 2:  # Ensure the point is in front of the vehicle
                    min_distance = distance
                    lookahead_point = (x, y, v)    # valid lookahead point found; store it
                    self.target_index = i

        return lookahead_point
    

    def get_speed_index(self, index, lookahead):

        anzahl_indizes = lookahead / self.distance_index
        
        
        speed_index = index - int(anzahl_indizes + 0.5) + self.speed_idx_diff

        if(speed_index < 0):
            speed_index = len(self.race_line) - abs(speed_index)
            
        if speed_index >=  len(self.race_line):
            speed_index = 0
        return speed_index


    def get_goalPoint(self):
        goalPt = self.race_line[0]

        currentX = self.current_pose.pose.position.x
        currentY = self.current_pose.pose.position.y

        current_orientation = self.current_pose.pose.orientation
        current_yaw = quat2euler([current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z])[2]

        # boolean variable to keep track of if intersections are found
        intersectFound = False  
        #lastFoundIndex= LFindex
    
        
        # output (intersections found) should be stored in arrays sol1 and sol2  
        # if two solutions are the same, store the same values in both sol1 and sol2  
        for i in range (self.target_index, len(self.race_line)):
            
            race_line_first = self.race_line[i]
            race_line_second = self.race_line[(i+1)% (len(self.race_line))]
            
            
            # subtract currentX and currentY from [x1, y1] and [x2, y2] to offset the system to origin  
            x1_offset = race_line_first[0]- currentX  
            y1_offset = race_line_first[1] - currentY  
            x2_offset = race_line_second[0] - currentX  
            y2_offset = race_line_second[1] - currentY  

            # calculate the discriminant using equations from the wolframalpha article
            dx = x2_offset - x1_offset
            dy = y2_offset - y1_offset
            dr = math.sqrt(dx**2 + dy**2)
            D = x1_offset*y2_offset - x2_offset*y1_offset
            discriminant = (self.lookahead_distance**2) * (dr**2) - D**2  
            
            
            



            if discriminant >= 0:
                
                # calculate the solutions
                sol_x1 = (D * dy + Helperfunctions.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_x2 = (D * dy - Helperfunctions.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
                sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2    

                # add currentX and currentY back to the solutions, offset the system back to its original position
                sol1 = [sol_x1 + currentX, sol_y1 + currentY]
                sol2 = [sol_x2 + currentX, sol_y2 + currentY] 

               # find min and max x y values
                minX = min(race_line_first[0], race_line_second[0])
                maxX = max(race_line_first[0], race_line_second[0])
                minY = min(race_line_first[1], race_line_second[1])
                maxY = max(race_line_first[1], race_line_second[1])
                
                
                
        
            

            # check to see if any of the two solution points are within the correct range
            # for a solution point to be considered valid, its x value needs to be within minX and maxX AND its y value needs to be between minY and maxY
            # if sol1 OR sol2 are within the range, intersection is found
                if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY) or (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY) :
                    intersectFound = True 

                    
                    
                    angle_to_sol1= math.atan2(sol1[1]- currentY,sol1[0]- currentX)  
                    angle_to_sol2= math.atan2(sol2[1]- currentY,sol2[0]- currentX)                      
                    angle_diff_sol1 = angle_to_sol1 - current_yaw
                    angle_diff_sol2 = angle_to_sol2 - current_yaw
                    
                    if abs(angle_diff_sol1) < math.pi /2 :
                        self.speed_idx = self.get_speed_index(i, self.lookahead_distance)
                        speed = self.race_line[self.speed_idx][2]
                        #goalPt = (sol1[0],sol1[1], race_line_first[2]) if (minX <= sol1[0] <= maxX) and (minY <= sol1[1] <= maxY) else (sol2[0],sol2[1], race_line_first[2])
                        goalPt = (sol1[0],sol1[1], speed) # x y v 
                    else: 
                        self.speed_idx = self.get_speed_index(i, self.lookahead_distance)
                        speed = self.race_line[self.speed_idx][2]
                        goalPt=(sol2[0],sol2[1], speed)
                    
                    self.target_index=i
                    if Helperfunctions.calculate_distance([goalPt[0], goalPt[1]], race_line_second) < Helperfunctions.calculate_distance([currentX, currentY], race_line_second):
                        self.target_index = i
                        break
                    else:
                        self.target_index+=1
                else:
                    foundIntersection = False
                    goalPt = self.race_line[self.target_index]

        # calc race_position idx in percent
        race_pos = int(self.target_index / len(self.race_line) * 100)
        # publish race_position
        self.race_position_pub.publish(Int16(data=race_pos))

        return goalPt
                    
        #if goalPt is None:
        #    # what to do when no point is found on 
        #    self.lookahead_distance += 0.5
        #    goalPt = self.get_goalPoint()
        #    
        #return goalPt


    def follow_race_line(self):
        if self.current_pose is None:
            return
        if self.target_index >= len(self.race_line)-2:
            self.target_index=0
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        self.calc_lookahead_based_on_crosserror(self.race_line, current_x, current_y)
        lookahead_point = self.get_goalPoint()

        ## get speed diff
        #set_speed = lookahead_point[2]
        set_speed = self.race_line[self.speed_idx-self.speed_idx_diff][2]
        current_speed = self.current_speed
        self.debug_pub.publish(Float32(data=current_speed-set_speed))

        if lookahead_point is None:
            return
        
        self.lookahead_point = lookahead_point
        target_x, target_y, target_v = lookahead_point
        

        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        current_orientation = self.current_pose.pose.orientation
        current_yaw = quat2euler([current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z])[2]
        alpha = angle_to_target - current_yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        # calculate steering angle according to wheelbase
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), self.lookahead_distance)

        # limit steering angle to 20°
        max_steering_angle = math.radians(38)
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))

        drive_msg = AckermannDriveStamped()
        speed_scale = (1- abs(self.cross_track_error * 0.5))
        if speed_scale < 0:
            speed_scale = 0.1
        drive_msg.drive.speed = float(target_v)
        drive_msg.drive.steering_angle = float(steering_angle) *1.1
        self.publisch_direction_message(steering_angle)
        self.drive_pub.publish(drive_msg)
        
    def publisch_direction_message(self, steering_angle):
        # configurate direction_msg
        aimed_angle = steering_angle

        direction_msg = PoseStamped()

        direction_msg.header.stamp = self.get_clock().now().to_msg()

        direction_msg.header.frame_id = "laser"

        direction_msg.pose.position = Point(x=0.0,y=0.0,z=0.0)

        q = Helperfunctions.quaternion_from_euler(0,0, aimed_angle)

        direction_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.direction_publischer.publish(direction_msg)



        return None


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
