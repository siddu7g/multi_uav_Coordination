#!/usr/bin/python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from mrs_msgs.msg import Reference
from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.srv import Vec1, Vec1Response

# Set filtering thresholds
MAX_THRESHOLD = 5.0  # Max allowable LiDAR range (meters)
MIN_DISTANCE = 0.5  # Min distance to avoid invalid readings
SAFE_DISTANCE = 1.6  # Distance threshold for obstacle avoidance
AVOIDANCE_OFFSET = 0.5  # Lateral deviation when avoiding obstacles

class SweepingGenerator:
    def __init__(self):
        rospy.init_node("sweeping_generator", anonymous=True)
        
        # Initialize parameters
        self.frame_id = rospy.get_param("~frame_id")
        self.center_x = rospy.get_param("~center/x")
        self.center_y = rospy.get_param("~center/y")
        self.center_z = 3.00  # Fixed navigation height
        self.dimensions_x = rospy.get_param("~dimensions/x")
        self.dimensions_y = rospy.get_param("~dimensions/y")
        self.timer_main_rate = rospy.get_param("~timer_main/rate")
        
        rospy.loginfo('[UAVNavigator]: Initialized parameters.')
        
        # ROS services and subscribers
        self.ss_start = rospy.Service('~start_in', Vec1, self.callback_start)
        self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)
        self.lidar_sub = rospy.Subscriber("/uav1/os_cloud_nodelet/points", PointCloud2, self.lidar_callback)
        
        self.is_initialized = True
        self.obstacle_detected = False
        self.obstacle_position = None  # Store obstacle position
        self.sweep_step_size = 1.0  # Default step size for sweeping
        
        rospy.spin()
    
    def lidar_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if not points:
            rospy.logwarn("No valid LiDAR points received.")
            return

        # Filter and compute minimum distance
        filtered_points = [
            (x, y, z) for x, y, z in points
            if MIN_DISTANCE <= np.sqrt(x**2 + y**2 + z**2) <= MAX_THRESHOLD
        ]

        if not filtered_points:
            rospy.loginfo("No obstacles within detection range.")
            self.obstacle_detected = False
            return

        # Find closest obstacle
        closest_obstacle = min(filtered_points, key=lambda p: np.sqrt(p[0]**2 + p[1]**2 + p[2]**2))
        min_distance = np.sqrt(closest_obstacle[0]**2 + closest_obstacle[1]**2 + closest_obstacle[2]**2)

        rospy.loginfo(f"[LiDAR]: Closest Obstacle at ({closest_obstacle[0]}, {closest_obstacle[1]}) - Distance: {min_distance:.3f}m")

        if min_distance < SAFE_DISTANCE:
            rospy.logwarn("Obstacle detected! Executing avoidance maneuver.")
            self.obstacle_detected = True
            self.obstacle_position = (closest_obstacle[0], closest_obstacle[1])  # Store obstacle's position
            self.avoid_obstacle()

    def avoid_obstacle(self):
        if not self.obstacle_position:
            rospy.logwarn("No obstacle position recorded, skipping avoidance maneuver.")
            return
        
        # Extract live x, y coordinates of the detected obstacle
        obstacle_x, obstacle_y = self.obstacle_position

        # Define avoidance maneuver: move left (decrease x by 0.5 meters)
        avoidance_x = obstacle_x - AVOIDANCE_OFFSET
        rospy.logwarn(f"Avoiding obstacle at ({obstacle_x}, {obstacle_y}), moving to ({avoidance_x}, {obstacle_y})")

        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True

        new_point = Reference()
        new_point.position.x = avoidance_x
        new_point.position.y = obstacle_y
        new_point.position.z = self.center_z
        new_point.heading = 0.0
        path_msg.path.points.append(new_point)

        try:
            response = self.sc_path.call(path_msg)
            if response.success:
                rospy.loginfo("Obstacle successfully avoided, resuming sweeping path.")
                self.obstacle_detected = False
                self.plan_sweeping_path(self.sweep_step_size)
            else:
                rospy.logerr("Obstacle avoidance failed, UAV might still be in danger!")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def plan_sweeping_path(self, step_size):
        rospy.loginfo(f'[UAVNavigator]: Planning sweeping path with step size: {step_size}')
        
        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        
        sign = 1.0  # Zigzag pattern
        for i in np.arange(-self.dimensions_x / 2.0, self.dimensions_x / 2.0, step_size):
            for j in np.arange(-self.dimensions_y / 2.0, self.dimensions_y / 2.0, step_size):
                if self.obstacle_detected:
                    rospy.logwarn("Obstacle detected, pausing sweeping path.")
                    return  # Stop execution if an obstacle is detected
                
                point = Reference()
                point.position.x = self.center_x + i
                point.position.y = self.center_y + j * sign
                point.position.z = self.center_z
                point.heading = 0.0
                path_msg.path.points.append(point)
                rospy.loginfo(f"Adding waypoint: ({point.position.x}, {point.position.y}, {point.position.z})")
            
            sign *= -1  # Switch direction for zigzag pattern
        
        try:
            response = self.sc_path.call(path_msg)
            if response.success:
                rospy.loginfo("Sweeping path executed successfully.")
            else:
                rospy.logerr("Path service did not succeed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def callback_start(self, req):
        if not self.is_initialized:
            return Vec1Response(False, "Node not initialized")
        
        rospy.loginfo("Received request to start sweeping path")
        self.plan_sweeping_path(req.goal)
        return Vec1Response(True, "Sweeping started")

if __name__ == '__main__':
    try:
        node = SweepingGenerator()
    except rospy.ROSInterruptException:
        pass