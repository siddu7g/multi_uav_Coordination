#!/usr/bin/python3

import rospy
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from mrs_msgs.msg import ControlManagerDiagnostics, Reference
from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.srv import Vec1, Vec1Response
from mrs_msgs.msg import UavStatusShort
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class MultiUAVCoordination:
    def __init__(self):
        # Get the namespace (UAV name)
        self.uav_name = rospy.get_namespace().strip('/')
        rospy.init_node("multi_uav_coordination", anonymous=True)

        # UAV names (for communication)
        self.uav_ids = ["uav1", "uav2", "uav3"]
        
        ## | --------------------- load parameters -------------------- |
        # Original trajectory parameters
        self.frame_id = rospy.get_param("~frame_id")
        self.center_x = rospy.get_param("~center/x")
        self.center_y = rospy.get_param("~center/y")
        self.center_z = rospy.get_param("~center/z")
        self.dimensions_x = rospy.get_param("~dimensions/x")
        self.dimensions_y = rospy.get_param("~dimensions/y")
        self.timer_main_rate = rospy.get_param("~timer_main/rate")
        self.trajectory_type = rospy.get_param("~trajectory_type", "sweep")
        
        # Disc detection parameters
        self.detection_threshold = rospy.get_param("~detection_threshold", 3)
        
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Initialized with {self.trajectory_type} trajectory')
        
        ## | ------------------- state variables -------------------- |
        self.disc_detected = False
        self.detection_count = 0
        self.bridge = CvBridge()
        self.control_manager_diag = None
        self.is_initialized = False
        
        # Track UAV positions
        self.uav_positions = {}
        self.disc_detector_uav = None
        
        ## | ----------------------- subscribers ---------------------- |
        # Original trajectory subscribers
        self.sub_control_manager_diag = rospy.Subscriber(
            "~control_manager_diag_in", 
            ControlManagerDiagnostics, 
            self.callbackControlManagerDiagnostics
        )
        
        # Camera feed for disc detection - connect to the rgbd camera's color image
        self.sub_camera = rospy.Subscriber(
            "/"+self.uav_name+"/rgbd/color/image_raw",  # Fixed camera topic from your description
            Image,
            self.callbackCamera
        )
        
        # Subscribe to position data from all UAVs using mrs_uav_status/uav_status_short
        # These topics are already shared via Nimbro
        for uav_id in self.uav_ids:
            rospy.Subscriber(
                f"/{uav_id}/mrs_uav_status/uav_status_short",
                UavStatusShort,
                self.callbackUavStatus,
                callback_args=uav_id
            )
            
            # Subscribe to disc detection notifications from other UAVs
            # These topics are shared via Nimbro's topic forwarding
            if uav_id != self.uav_name:
                rospy.Subscriber(
                    f"/{uav_id}/disc_detection",
                    Bool,
                    self.callbackDiscDetection,
                    callback_args=uav_id
                )
        
        ## | ----------------------- publishers ----------------------- |
        # Publisher to notify other UAVs when disc is detected
        # This should be forwarded by Nimbro to other UAVs
        self.pub_disc_detection = rospy.Publisher(
            "/"+self.uav_name+"/disc_detection", 
            Bool, 
            queue_size=10
        )
        
        # Visualization publisher (local only, not shared)
        self.pub_visualization = rospy.Publisher(
            "~visualization_out",
            Image,
            queue_size=10
        )
        
        ## | --------------------- service servers -------------------- |
        # Service name is directly 'start_in' based on your requirements
        self.ss_start = rospy.Service('~start_in', Vec1, self.callbackStart)

        ## | --------------------- service clients -------------------- |
        self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)

        ## | ------------------------- timers ------------------------- |
        self.timer_main = rospy.Timer(rospy.Duration(1.0/self.timer_main_rate), self.timerMain)

        ## | -------------------- spin till the end ------------------- |
        self.is_initialized = True
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Ready')
        rospy.spin()
        
    ## | ------------------------- methods ------------------------ |
    
    def detectDisc(self, image):
        """
        Detect gray disc in the image using HSV color space
        Returns: (found, x, y, width, height)
        """
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define the range for gray color detection in HSV space
        lower_gray = np.array([0, 0, 50])  # Lower bound (dark gray)
        upper_gray = np.array([180, 50, 200])  # Upper bound (light gray)
        
        # Create a mask to detect gray regions
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        
        # Perform morphological operations to remove noise and enhance the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours to find suitable gray objects
        for contour in contours:
            # Calculate the bounding box and area
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            
            # Filter based on minimum area
            if area > 500:  # Minimum area threshold (adjust as needed)
                # Log detection
                rospy.loginfo(f"[SweepingGenerator-{self.uav_name}]: Potential disc detected at ({x}, {y}) with size {w}x{h}")
                return True, x, y, w, h
                
        # No disc found
        return False, 0, 0, 0, 0
        
    def broadcastDiscDetection(self):
        """Broadcast disc detection to other UAVs"""
        msg = Bool()
        msg.data = True
        self.pub_disc_detection.publish(msg)
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Broadcasting disc detection!')

    def planCircleTrajectory(self, radius_factor=1.0):
        """Plan a circular trajectory"""
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Planning circular trajectory')
        
        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        
        # Use dimensions_x as radius
        radius = self.dimensions_x/2.0 * radius_factor
        
        # Number of points in the circle
        num_points = 30
        
        # Create circle points
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            
            point = Reference()
            point.position.x = self.center_x + radius * math.cos(angle)
            point.position.y = self.center_y + radius * math.sin(angle)
            point.position.z = self.center_z
            # Set heading tangent to the circle
            point.heading = angle + math.pi/2  # Tangent to the circle
            
            path_msg.path.points.append(point)
            
        return path_msg
        
    def planRectangleTrajectory(self, scale_factor=1.0):
        """Plan a rectangular trajectory"""
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Planning rectangular trajectory')
        
        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        
        # Scale the rectangle dimensions
        width = self.dimensions_x * scale_factor
        height = self.dimensions_y * scale_factor
        
        # Define the four corners of the rectangle
        corners = [
            (self.center_x - width/2, self.center_y - height/2),  # Bottom left
            (self.center_x + width/2, self.center_y - height/2),  # Bottom right
            (self.center_x + width/2, self.center_y + height/2),  # Top right
            (self.center_x - width/2, self.center_y + height/2),  # Top left
            (self.center_x - width/2, self.center_y - height/2)   # Back to start
        ]
        
        # Add points for the rectangle
        for i, (x, y) in enumerate(corners):
            point = Reference()
            point.position.x = x
            point.position.y = y
            point.position.z = self.center_z
            
            # Set heading based on direction of travel
            if i < len(corners) - 1:
                next_x, next_y = corners[i+1]
                heading = math.atan2(next_y - y, next_x - x)
            else:
                heading = 0.0
                
            point.heading = heading
            path_msg.path.points.append(point)
            
        return path_msg

    def planSweepPath(self, step_size):
        """Plan a sweeping pattern"""
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Planning sweeping path with step size {step_size}')

        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True

        sign = 1.0

        # Fill in the path with a sweeping pattern
        for i in np.arange(-self.dimensions_x/2.0, self.dimensions_x/2.0, step_size):
            for j in np.arange(-self.dimensions_y/2.0, self.dimensions_y/2.0, step_size):
                # Create a reference point
                point = Reference()
                point.position.x = self.center_x + i
                point.position.y = self.center_y + j*sign
                point.position.z = self.center_z
                point.heading = 0.0

                path_msg.path.points.append(point)

            if sign > 0.0:
                sign = -1.0
            else:
                sign = 1.0

        return path_msg
        
    def planPathToPosition(self, target_x, target_y, target_z=None):
        """Plan a path to fly to a specific position"""
        if target_z is None:
            target_z = self.center_z
            
        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Planning path to position [{target_x}, {target_y}, {target_z}]')
        
        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True
        
        # Create a single point for direct flight
        point = Reference()
        point.position.x = target_x
        point.position.y = target_y
        point.position.z = target_z
        point.heading = 0.0  # Face forward
        
        path_msg.path.points.append(point)
        
        return path_msg

    ## | ------------------------ callbacks ----------------------- |
    
    def callbackCamera(self, data):
        """Process camera frames to detect discs"""
        # Skip processing if we've already detected a disc
        # or if we're responding to another UAV's detection
        if self.disc_detected or self.disc_detector_uav is not None:
            return
            
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Detect disc
            found, x, y, w, h = self.detectDisc(cv_image)
            
            # Draw detection visualization
            if found:
                # Visualize the detection with rectangle
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image, (x + w//2, y + h//2), 5, (0, 0, 255), -1)
                cv2.putText(cv_image, f"Disc ({x}, {y})", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Count consecutive detections
                self.detection_count += 1
                
                # Confirm detection after threshold
                if self.detection_count >= self.detection_threshold and not self.disc_detected:
                    self.disc_detected = True
                    self.disc_detector_uav = self.uav_name  # Mark self as the detector
                    rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Disc detected!')
                    self.broadcastDiscDetection()
            else:
                # Reset detection counter if no disc is found
                self.detection_count = 0
            
            # Add status text
            status = "DISC DETECTED" if self.disc_detected else "SEARCHING"
            if self.disc_detector_uav and self.disc_detector_uav != self.uav_name:
                status = f"FORMATION WITH {self.disc_detector_uav}"
                
            cv2.putText(cv_image, f"UAV: {self.uav_name} - {status}", (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub_visualization.publish(viz_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"[SweepingGenerator-{self.uav_name}]: {e}")
    
    def callbackUavStatus(self, msg, uav_id):
        """Handle status updates from UAVs"""
        # Store position for each UAV using the mrs_uav_status topic
        self.uav_positions[uav_id] = {
            'x': msg.odom_x,  # Using pose array from UavStatusShort
            'y': msg.odom_y,
            'z': msg.odom_z,
            'heading': msg.odom_hdg
        }
        
        # If we're following a UAV that detected a disc, update our path
        if self.disc_detector_uav == uav_id:
            # Update path to follow the detecting UAV
            self.flyToDetectingUAV()
            
    def callbackDiscDetection(self, msg, uav_id):
        """Handle disc detection messages from other UAVs"""
        if msg.data and not self.disc_detected and self.disc_detector_uav is None:
            rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Received disc detection from {uav_id}')
            self.disc_detector_uav = uav_id
            
            # Fly to the position of the detecting UAV
            self.flyToDetectingUAV()
    
    def flyToDetectingUAV(self):
        """Navigate to the position of the UAV that detected the disc, maintaining formation"""
        # Make sure we have position data for the detecting UAV
        if self.disc_detector_uav in self.uav_positions:
            pos = self.uav_positions[self.disc_detector_uav]
            
            # Get the UAV IDs that need to respond
            responding_uavs = [uav for uav in self.uav_ids if uav != self.disc_detector_uav]
            
            # Find my index in the responding UAVs list
            if self.uav_name in responding_uavs:
                my_index = responding_uavs.index(self.uav_name)
                formation_size = len(responding_uavs)
                
                # Calculate formation position (2 meters apart in opposite directions)
                # Use a simple perpendicular line formation
                if formation_size == 1:
                    # Only one responding UAV, place it 2 meters to the right of detecting UAV
                    offset_x = 3.5
                    offset_y = 0.0
                elif formation_size == 2:
                    # Two responding UAVs, place them on opposite sides
                    if my_index == 0:
                        # First UAV goes 2 meters to the right
                        offset_x = 3.5
                        offset_y = 0.0
                    else:
                        # Second UAV goes 2 meters to the left
                        offset_x = -3.5
                        offset_y = 0.0
                else:
                    # More than 2 UAVs, distribute them evenly in a circle
                    angle = 2 * math.pi * my_index / formation_size
                    offset_x = 3.0 * math.cos(angle)
                    offset_y = 3.0 * math.sin(angle)
                
                # Plan path to the formation position relative to detecting UAV
                target_x = pos['x'] + offset_x
                target_y = pos['y'] + offset_y
                target_z = pos['z']
                
                rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Flying to formation position ({offset_x}, {offset_y}) relative to {self.disc_detector_uav}')
                
                # Create the path message
                path_msg = self.planPathToPosition(target_x, target_y, target_z)
                
                # Call path service
                try:
                    response = self.sc_path.call(path_msg)
                    if response.success:
                        rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Formation position set, offset: ({offset_x}, {offset_y}) from {self.disc_detector_uav}')
                    else:
                        rospy.logwarn(f'[SweepingGenerator-{self.uav_name}]: Path setting failed: {response.message}')
                except Exception as e:
                    rospy.logerr(f'[SweepingGenerator-{self.uav_name}]: Error calling path service: {e}')
            else:
                # I am the detecting UAV, no need to move
                rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: I am the detecting UAV, maintaining position')
        else:
            rospy.logwarn(f'[SweepingGenerator-{self.uav_name}]: No position data for {self.disc_detector_uav}')

    def callbackControlManagerDiagnostics(self, msg):
        if not self.is_initialized:
            return

        rospy.loginfo_once(f'[SweepingGenerator-{self.uav_name}]: Getting ControlManager diagnostics')
        self.control_manager_diag = msg

    def callbackStart(self, req):
        if not self.is_initialized:
            return Vec1Response(False, "not initialized")
            
        # Skip trajectory generation if we've detected a disc or are following another UAV
        if self.disc_detected or self.disc_detector_uav is not None:
            return Vec1Response(False, "Currently handling disc detection")

        # Get the parameter value from the service call
        param_value = req.goal
        if param_value <= 0:
            param_value = 1.0  # Default value if non-positive

        # Plan the appropriate trajectory based on the trajectory type
        if self.trajectory_type == "circle":
            path_msg = self.planCircleTrajectory(param_value)
        elif self.trajectory_type == "rectangle":
            path_msg = self.planRectangleTrajectory(param_value)
        else:  # Default to sweep
            path_msg = self.planSweepPath(param_value)

        # Call the path service
        try:
            response = self.sc_path.call(path_msg)
        except Exception as e:
            rospy.logerr(f'[SweepingGenerator-{self.uav_name}]: Path service not callable: {str(e)}')
            return Vec1Response(False, "service call failed")

        if response.success:
            rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Path set with {self.trajectory_type} trajectory')
        else:
            rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Path setting failed: {response.message}')

        return Vec1Response(response.success, response.message)

    ## | ------------------------- timers ------------------------- |

    def timerMain(self, event=None):
        if not self.is_initialized:
            return

        rospy.loginfo_once(f'[SweepingGenerator-{self.uav_name}]: Main timer spinning')
        
        # Re-broadcast detection periodically if we found a disc
        if self.disc_detected:
            self.broadcastDiscDetection()
            
        # Periodically update path if following another UAV
        if self.disc_detector_uav is not None and self.disc_detector_uav in self.uav_positions:
            self.flyToDetectingUAV()

        # Handle control manager diagnostics
        if isinstance(self.control_manager_diag, ControlManagerDiagnostics):
            if self.control_manager_diag.tracker_status.have_goal:
                status = "DISC DETECTED" if self.disc_detected else "SEARCHING"
                if self.disc_detector_uav:
                    status = f"FOLLOWING {self.disc_detector_uav}"
                rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Status: {status}')
            else:
                rospy.loginfo(f'[SweepingGenerator-{self.uav_name}]: Waiting for command')

if __name__ == '__main__':
    try:
        node = MultiUAVCoordination()
    except rospy.ROSInterruptException:
        pass