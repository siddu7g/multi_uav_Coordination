# Trajectory Generation and Multi UAV Communication using ROS Nodes

## How to start

Step 1

cd into the MRS_UAV three_drones system 
```bash
./tmux/start.sh
```
Step2 

Ctrl+t in the terminal -

```bash
roscd example_multi_uav_coordination  
roslaunch example_multi_uav_coordination multi_uav_coordination.launch
```


When ready, call the service prepared in the bottom terminal window for generating the trajectory:
```bash
rosservice call /$UAV_NAME/multi_uav_coordination/start 3.0
```
uav1
uav2
uav3

Description

Key Features:

Shared Odometry & Telemetry:
Subscribes to UAV state topics (mrs_uav_status/uav_status_short) to track the position, altitude, and heading of all drones in the swarm.
Shares detection messages (/disc_detection) between UAVs using ROS publishers and subscribers, enabling collaborative awareness.

Vision-based Detection:

Uses the onboard RGBD camera feed and OpenCV (cv2) to detect gray circular discs in the environment.
Once confirmed, detection is broadcast to other UAVs in the network.

Path Planning & Trajectories:

Implements multiple trajectory planners:
Circle trajectory – UAV follows a circular path around a defined center.
Rectangle trajectory – UAV scans an area in a rectangular pattern.
Sweep trajectory – UAV performs a sweeping lawnmower-style scan of an area.
Targeted path – UAV flies directly to a given (x, y, z) coordinate.
All paths are generated as PathSrvRequest messages and executed via ROS services.

Multi-Agent Coordination:
When a disc is detected by one UAV, other UAVs automatically reposition into formation relative to the detecting UAV.
Formation offsets are dynamically calculated (line or circular distribution depending on swarm size).
Continuous re-broadcast ensures swarm-wide synchronization.

![Traget Search](videos/screenshot.png)

Visualization & Diagnostics:

Publishes annotated camera images with detection overlays for debugging and visualization.
Logs UAV status, detection events, and path execution progress.
