# Lyapunov Adaptive Transformer (LyAT) for Quadrotor Control

> Real-time adaptive trajectory tracking with guaranteed stability for autonomous UAVs

## Overview

Implements a Lyapunov-based adaptive controller for quadrotor trajectory tracking under 
model uncertainties. 

---

### Frame Conventions
- **APark Frame**: Local park coordinates (rotated ENU, origin at field center)
- **UTM**: Universal Transverse Mercator for GPS conversion
- **ENU**: East-North-Up (MAVROS local frame)
- **NED**: North-East-Down (PX4 body frame)

---

## Prerequisites

### Required Software
- Ubuntu 22.04 + ROS2 Humble
- PX4-Autopilot 1.15 (https://github.com/PX4/PX4-Autopilot/releases/tag/v1.15.4)
- Mavros
- Gazebo Fortress (https://gazebosim.org/docs/fortress/install_ubuntu/)
- QGroundControl (https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
---

### Python dependencies
```bash
pip3 install torch numpy scipy geodesy transforms3d
```

### Required Packages
```bash
cd ~/ros2_ws/src
git clone https://github.com/UFL-Autonomy-Park/aero_common
git clone https://github.com/Jordan-Insinger/lyapunov_adaptive_transformer
cd aero_common
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select px4_safety_lib autonomy_park_viz px4_telemetry px4_teleop lyapunov_adaptive_transformer
source install/setup.bash
```

## Usage

### Simulation (Gazebo)
```bash
# Open QGC

# Terminal 1: Launch PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: Start MAVROS
ros2 launch px4_telemetry px4_sim.launch

# Terminal 3: Launch Telemetry Node
ros2 launch px4_telemetry astro_sim.launch.py

# Terminal 4: Launch Control Node
ros2 launch lyapunov_adaptive_transformer lyapunov_adaptive_transforerm.launch.py
```

### Trajectory Configuration
Edit `config.json`:
```json
{
  "T_final": 30.0,        // Mission duration (s)
  "dt": 0.01,             // Control period (s)  
  "n_states": 6,          // [x,y,z,vx,vy,vz]

}
```

---

## Results

Achieved --RESULTS HERE-- RMS tracking error on --TRAJECTORY HERE--


### Convergence Behavior
![Plot showing Lyapunov function decreasing over time](docs/images/lyapunov_convergence.png)


## Citation

If you use this work, please cite:
```bibtex
@misc{yourname2024lyat,
  author = {Your Name},
  title = {Lyapunov Adaptive Transformer for Quadrotor Control},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/yourusername/lyapunov_adaptive_transformer}
}
```

**Related Publications:**
- [Link to your lab's paper if published]
- [Link to thesis/technical report]

---

## Acknowledgments

This work was developed at [Your Lab Name], [University]. Special thanks to:
- Dr. [Advisor Name] for theoretical guidance
- [Lab members] for hardware integration support  
- PX4 and ROS2 communities

---
