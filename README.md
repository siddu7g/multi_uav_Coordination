# Sweeping Generator

## How to start

```bash
./tmux/start.sh
```

roscd example_multi_uav_coordination
  
roslaunch example_multi_uav_coordination multi_uav_coordination.launch

When ready, call the service prepared in the bottom terminal window:
```bash
rosservice call /$UAV_NAME/multi_uav_coordination/start 3.0
```

