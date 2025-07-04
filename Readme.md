# 🛸 UAV Deconfliction System – ROS 2 (Humble) + Plotly 4D Visualization

A modular and scalable airspace conflict detection system that simulates UAV trajectories, detects potential collisions, and visualizes them in 4D (space + time).

---

## 🚀 Features

- Real-time conflict detection between a primary drone and multiple simulated drones  
- Custom ROS 2 messages and services  
- Interpolated spatiotemporal path comparison  
- JSON-based logging and 3D/4D interactive Plotly visualization  
- Modular and extendable architecture  

---
---

## 🧪 Prerequisites

- ROS 2 Humble
- Python 3.10+
- Plotly (`pip install plotly`)
- Colcon (`sudo apt install python3-colcon-common-extensions`)

---

## 🔧 Build Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/Akash-Potti/Drone-Deconfliction-System.git

2. Build the workspace
   ```bash
    cd ~/Drone-Deconfliction-System-master
    colcon build
3. Source the workspace
    ```bash
    source install/setup.bash
---
## 🚦 Running the System

1. Run All 3 ros nodes at once
   ```bash
   ros2 launch deconfliction_system deconfliction_system.launch.py

2. Run 3D visualisation
   ```bash
   python Visualisation.py
   or
   python3 Visualisation.py
   
3. Run 4D visualisation
   ```bash
   python animated_4d_visualization.py
   or
   python3 animated_4d_visualization.py

---
## ✅ Status

- [x] **Phase 0**: ROS 2 node setup
- [x] **Phase 1**: Deconfliction engine with interpolation + conflict logging
- [x] **Phase 2**: 4D Plotly animation with conflict zones
- [x] **Query service interface**
- [ ] **Future**: Dynamic replanning / Gazebo PX4 integration
