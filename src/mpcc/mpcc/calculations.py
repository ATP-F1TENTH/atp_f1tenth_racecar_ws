
import numpy as np
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from tf2_ros.buffer import Buffer


class Localization(None):

    def __init__(self):
        pass


    def find_theta(current_position, track_center, traj_breaks, track_width, last_closest_idx, n_search_region):
        """
        Searching and returning index of centerline point (theta) being closest to the current position.
            => From Liniger MPCC repo: https://github.com/alexliniger/MPCC/blob/master/Matlab/findTheta.m
        """

        pos_x = current_position[0]
        pos_y = current_position[1]
        track_x = track_center[0, :]
        track_y = track_center[1, :]
        n_track = len(track_x)

        # Determine indices that are investigated
        start = last_closest_idx - 5
        search_region = np.asarray([x for x in range(start, start + n_search_region)], dtype=int)

        # Wrap if necessary
        if last_closest_idx > n_track - 20 or last_closest_idx <= 5:
            search_region[search_region < 1] += n_track
            search_region[search_region > n_track] -= n_track

        # Compute squared distance to the current location
        distance_x = track_x[search_region] - pos_x
        distance_y = track_y[search_region] - pos_y
        squared_distance_array = distance_x**2 + distance_y**2

        # Find closest point
        min_index = search_region[np.argmin(squared_distance_array)]

        # If distance is too large, perform global search with respect to track width
        if (np.sqrt(squared_distance_array[min_index]) > (track_width * 1.25) / 2):
            distance_x2 = track_x - pos_x
            distance_y2 = track_y - pos_y
            squared_distance_array2 = distance_x2**2 + distance_y2**2
            min_index = np.argmin(squared_distance_array2)
        
        # Check!
        next_idx = min_index+1 if (min_index+1 < n_track) else 0
        prev_idx = min_index-1 if (min_index-1 >= 0) else n_track-1

        # Compute theta based on inner product projection
        closest_idx = min_index
        cosinus = np.dot(np.array([pos_x, pos_y]) - track_center[:, min_index], 
                        track_center[:, prev_idx] - track_center[:, min_index])

        min_index2 = prev_idx if cosinus > 0 else min_index
        if cosinus <= 0:
            min_index = next_idx

        if np.any(squared_distance_array[min_index2] != 0):
            norm_proj = np.linalg.norm(np.array([pos_x, pos_y]) - track_center[:, min_index2])
            norm_edge = np.linalg.norm(track_center[:, min_index] - track_center[:, min_index2])

            cosinus = np.dot(np.array([pos_x, pos_y]) - track_center[:, min_index2], 
                            track_center[:, min_index] - track_center[:, min_index2]) / (norm_proj * norm_edge)
        else:
            cosinus = 0

        theta = traj_breaks[0, min_index2]
        theta += cosinus * np.linalg.norm(np.array([pos_x, pos_y]) - track_center[:, min_index2])

        return theta, closest_idx
    

    def findNearestPoint(self, frameNames, racelineIndex, raceline) -> tuple[float, float, float, float]:
        """
        Searching the centerline point which is closest to the current position and returning its
        coordinates, index and the necessary steering angle phi to reach the point.
        """

        # get current position on map
        tf_buffer = Buffer()
        try:
            lookupTransformBuffer = tf_buffer.lookup_transform(frameNames['map'], frameNames['vehicle'], rclpy.time.Time())
            x_vehicle_map = lookupTransformBuffer.transform.translation.x
            y_vehicle_map = lookupTransformBuffer.transform.translation.y
        except:
            msg = f"No valid transform from {frameNames['map']} to {frameNames['vehicle']}. Function aborted."
            print(msg, flush=True)
            return

        # find nearest point on raceline
        best_diff = 1_000_000
        best_idx = racelineIndex
        found_better_point = False
        for i in range(len(raceline.x)):
            idx = (racelineIndex + i) % len(raceline.x)
            diff_x = raceline.x[idx]-x_vehicle_map
            diff_y = raceline.y[idx]-y_vehicle_map
            len_diff = math.sqrt(diff_x*diff_x + diff_y*diff_y)
            if (len_diff < best_diff):
                best_diff = len_diff
                best_idx = idx
                found_better_point = True
            elif (self.racelineIndex > -1 and found_better_point):
                break   # nearest point already found => moving away again

        # tranformation => coordinate system map to vehicle
        x = raceline.x[best_idx]
        y = raceline.y[best_idx]
        x, y = self.transformPoint(x, y, frameNames['map'], frameNames['vehicle'])

        # calculate steering angle phi to point on raceline
        b = x
        c = math.sqrt(x*x + y*y)
        factor = 1.0 if (y < 0) else -1.0   # determine stering direction
        phi = factor * math.acos(b/c) if (c != 0.0) else 0.0

        return (x, y, best_diff, phi)


    def transformPoint(self, x:float, y:float, from_frame: str, target_frame:str) -> tuple[float, float]:
        point_in_map = PoseStamped()
        point_in_map.pose.position.x = x
        point_in_map.pose.position.y = y
        point_in_map.header.frame_id = from_frame
        point_in_map.header.stamp = rclpy.time.Time().to_msg() #get the newest transform
        point_in_vehicle = self.tf_buffer.transform(point_in_map, target_frame)
        
        x = point_in_vehicle.pose.position.x
        y = point_in_vehicle.pose.position.y

        return (x, y)