# px4-mark-position

## Description

# 🔧 Setup & Build
MacOS
```bash
   ./setup/macos.sh
```

Linux <br>
1. Setup repositories.
```bash
   ./setup/global.sh
```
2. Go to PX4 directory.
```bash
   cd PX4-Autopilot
```
3. Follow [official setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu).

## Run
1. Run PX4 + gazebo
```bash
   ./px4_start.sh [drone mode]
```
2. Upload params in PX4
3. Run python script to send position from camera
```bash
   python3 src/send_position.py
```
4. Run mission script
```bash
   python3 src/mission.py
```

## Drone modes
### sim_cam_up
Drone: gz_x500_mono_cam_up <br>
World: aruco_roof

### sim_cam_down
Drone: gz_x500_mono_cam_down <br>
World: aruco_floor
