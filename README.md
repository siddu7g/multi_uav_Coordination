# Sweeping Generator

## How to start

Step 1

cd into the MRS_UAV three_drones system 
```bash
./tmux/start.sh
```
Step2 

Ctrl+t in the terminal -
roscd example_multi_uav_coordination  
roslaunch example_multi_uav_coordination multi_uav_coordination.launch

When ready, call the service prepared in the bottom terminal window for generating the trajectory:
```bash
rosservice call /$UAV_NAME/multi_uav_coordination/start 3.0
```
uav1
uav2
uav3
