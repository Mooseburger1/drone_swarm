# `drone_swarm` — 6-DOF Quadrotor Swarm Formation Control

This repository contains our ROS 2 Jazzy simulation and control package for **EN.605.613 (Introduction to Robotics)**: ***6-DOF Kinematic and Dynamic Modeling of a Decentralized Quadrotor Swarm for Rigid Formation Maintenance***.

To ensure everyone on the team has the exact same ROS 2 Jazzy environment regardless of host operating system (**Linux, Windows, or macOS**), the workspace is containerized with Docker and supports both native Linux desktop GUI windows (via X11) and a browser-based VNC desktop (for Windows/macOS).

---

## 1. System Overview & Mathematical Foundation

The `drone_swarm` package models, simulates, and visualizes a decentralized swarm of three 6-DOF quadrotor drones (`drone_1`, `drone_2`, `drone_3`) operating in a shared 3D `world` frame:

* **Homogeneous Transformations (`SE(3)`):** Each quadrotor's 6-DOF pose `[x_i, y_i, z_i, roll_i, pitch_i, yaw_i]` is represented by a 4×4 homogeneous transformation matrix `T_i^0` in `SE(3)`. Neighbor `j`'s relative pose in Drone `i`'s tilted body frame is computed analytically via `T_j^i = (T_i^0)^(-1) * T_j^0` (`drone_swarm/kinematics.py`).
* **Velocity Kinematics & Jacobians:** Body angular velocities `[p_i, q_i, r_i]^T` are mapped to ZYX Euler angle rates `[roll_dot_i, pitch_dot_i, yaw_dot_i]^T` via the rotational velocity Jacobian `W(roll_i, pitch_i)` while clamping pitch within `±35°` to avoid the `±90°` gimbal-lock singularity. Pairwise inter-drone distance rates are computed via the formation rigidity Jacobian projection `d_dot_ij = u_ij^T * (v_j - v_i)`.
* **Decentralized Parallel Virtual Spring-Damper Control:** Every pair of drones is coupled by a parallel Kelvin–Voigt virtual spring-damper link (`F_ij = k_s * (d_ij - d*) + c_d * d_dot_ij`) targeting an equilateral triangle with separation `d* = 2.0 m` and a short-range repulsive safety barrier enforcing `d_min > 0.6 m` (`drone_swarm/formation_control.py`).
* **Underactuated 6-DOF Attitude Inverse Kinematics:** Because a quadrotor's rotors only thrust along its local `+Z_i` axis, desired 3D world accelerations (plus gravity compensation) are inverted in closed form into total upward thrust `T_des` and target roll/pitch tilt angles `(roll_des, pitch_des)`.
* **6-DOF Newton–Euler Rigid-Body Dynamics:** A 50 Hz simulator (`drone_swarm/dynamics.py` & `drone_swarm/swarm_sim_node.py`) integrates translational and rotational equations of motion, spins all 12 continuous propeller joints, and broadcasts live `/tf` and `/drone_i/joint_states` messages to RViz2.

---

## 2. Repository & Package Structure

The Python codebase follows the **Google Python Style Guide (`go/pystyle`)** with modular separation between pure math/physics libraries and thin ROS 2 node wrappers:

```text
drone_swarm/                              
├── Dockerfile                            
├── docker-compose.yml                    
├── .gitignore                            
├── .gitattributes                        
├── README.md                             
└── src/
    └── drone_swarm/                      # ROS 2 python package
        ├── config/
        │   ├── swarm.rviz                # Pre-configured multi-drone + markers RViz2 layout
        │   └── urdf_test.rviz            # Pre-configured single-drone RViz2 layout
        ├── launch/
        │   ├── swarm_full.launch.py      # Full closed-loop sim + controller + RViz2
        │   ├── swarm_sim.launch.py       # Open-loop 6-DOF physics sim + RViz2
        │   ├── swarm_preview.launch.py   # Static 3-drone triangle + RViz2
        │   └── urdf_test.launch.py       # Single-drone URDF preview + RViz2
        ├── urdf/
        │   └── quadrotor.urdf            # 3D quadrotor URDF (hub, nose, 4 arms, 4 rotors)
        ├── drone_swarm/
        │   ├── __init__.py
        │   ├── constants.py              # Physical, geometric, and formation constants
        │   ├── kinematics.py             # SO(3), SE(3), Jacobian, and attitude IK math
        │   ├── dynamics.py               # 6-DOF rigid-body state & Newton-Euler step
        │   ├── formation_control.py      # virtual spring-damper & waypoint control law
        │   ├── swarm_markers.py          # RViz2 MarkerArray builder for spring links & labels
        │   ├── launch_utils.py           # Reusable ROS 2 launch node factory functions
        │   ├── swarm_sim_node.py         # 50 Hz ROS 2 6-DOF simulator node (swarm_sim)
        │   └── swarm_controller_node.py  # 50 Hz ROS 2 formation controller (swarm_controller)
        ├── package.xml
        └── setup.py
```

---

## 3. Prerequisites & First-Time Setup

### Prerequisites
1. **[Git](https://git-scm.com/downloads)** installed on your host machine.
2. **[Docker Desktop](https://docs.docker.com/get-docker/)** (or Docker Engine + Docker Compose on Linux) installed and running.
   * *Recommended Docker resources:* At least **8 GB RAM** and **4 CPU cores**.

### Step 1: Clone the Repository
Open a terminal on your computer (**Terminal** on Linux/macOS, **PowerShell** on Windows) and run:
```bash
git clone <YOUR_GITHUB_REPO_URL>/drone_swarm.git
cd drone_swarm
```

### Step 2: Build and Start the Docker Container
* **On Linux (enables native X11 GUI windows like RViz2 directly on your desktop):**
  ```bash
  xhost +local:
  docker compose up -d --build
  ```
* **On Windows / macOS:**
  ```bash
  docker compose up -d --build
  ```
> **Note:** The first build takes 5–10 minutes to download the base ROS 2 Jazzy image and install packages. Subsequent starts (`docker compose up -d`) take only a few seconds.

### Step 3: Enter the Container & Build the Workspace
1. Open an interactive shell inside the running container:
   ```bash
   docker compose exec drone_swarm bash
   ```
2. Inside the container (`/ros2_ws`), build the package with `--symlink-install`:
   ```bash
   cd /ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

> **Note on GUI Windows (`rviz2`):**
> * **Linux Hosts:** After running `xhost +local:` on your host, every launch file automatically opens **RViz2 natively on your desktop**.
> * **Windows / macOS Hosts:** Open **[http://127.0.0.1:6080/](http://127.0.0.1:6080/)** in your browser to view the container's Ubuntu VNC desktop where RViz2 opens automatically (or run an X server such as VcXsrv/XQuartz on your host).

---

## 4. Running the Simulations & Interactive Demos

Every launch file in `drone_swarm` **automatically launches RViz2** with its pre-configured `.rviz` layout.

### A. Full Closed-Loop Swarm Simulation (`swarm_full.launch.py`) — *Main Demo*
Launches all 3 `robot_state_publisher` instances, the 50 Hz 6-DOF physics simulator (`swarm_sim`), the 50 Hz virtual spring-damper formation controller (`swarm_controller`), and **RViz2**:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch drone_swarm swarm_full.launch.py
```

**What happens on startup:**
1. The 3 quadrotors spawn at scattered positions and altitudes (`(1.8, 0.4, 0.6)`, `(-1.2, 1.6, 1.4)`, `(-0.9, -1.5, 0.9)`).
2. The parallel virtual spring-dampers immediately engage—tilting the drones in roll and pitch to pull/push them into a rigid **2.00 m equilateral triangle** hovering at `z = 1.0 m`.
3. Live 3D spring-damper links (`/swarm_markers`) connect the drones with floating distance readouts (`d_12`, `d_13`, `d_23`):
   * **Green line:** Link is at equilibrium (`2.00 m ± 0.08 m`).
   * **Red line:** Spring is compressed (`d_ij < 1.92 m`), pushing the drones apart.
   * **Blue line:** Spring is stretched (`d_ij > 2.08 m`), pulling the drones together.

#### Interactive Controls (Run in a Second Container Terminal while `swarm_full.launch.py` is Running)

* **1. Fly the Swarm Using RViz2 "2D Goal Pose":**
  Click **2D Goal Pose** in the top RViz2 toolbar, then click and drag anywhere on the ground grid. The golden target arrow moves to the clicked location/heading, and all 3 drones tilt and fly across the map while maintaining their 2.00 m triangle formation.
* **2. Command a 3D Waypoint + Altitude via CLI:**
  You can also publish a 3D goal pose (including target altitude `z`) directly to `/goal_pose`:
  ```bash
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 2.0, y: 1.5, z: 1.8}, orientation: {w: 1.0}}}"
  ```
* **3. Inject an External Wind-Gust Disturbance (`/swarm_disturb`):**
  Applies an instantaneous velocity impulse of `[1.8, -1.5, 0.8] m/s` to `drone_1` so you can watch the virtual spring-dampers absorb the shock and restore the 2.00 m triangle:
  ```bash
  ros2 service call /swarm_disturb std_srvs/srv/Trigger
  ```
* **4. Reset the Swarm to Scattered Positions (`/swarm_reset`):**
  ```bash
  ros2 service call /swarm_reset std_srvs/srv/Trigger
  ```

---

### B. Other Launch Modes (For Step-by-Step Testing)

* **Open-Loop 6-DOF Physics Simulator Only (`swarm_sim.launch.py`):**
  Runs the 6-DOF physics simulator without the outer-loop formation controller:
  ```bash
  ros2 launch drone_swarm swarm_sim.launch.py
  ```
  Test underactuated roll/pitch flight manually from another terminal:
  ```bash
  ros2 topic pub --once /drone_1/cmd_attitude geometry_msgs/msg/Twist "{linear: {z: 9.85}, angular: {x: 0.0, y: 0.15, z: 0.0}}"
  ```
* **Static 3-Drone Formation Preview (`swarm_preview.launch.py`):**
  Spawns all 3 drones at static equilibrium TF coordinates in the `world` frame:
  ```bash
  ros2 launch drone_swarm swarm_preview.launch.py
  ```
* **Single-Drone URDF Preview (`urdf_test.launch.py`):**
  Spawns a single quadrotor in the `base_link` frame to inspect the URDF geometry:
  ```bash
  ros2 launch drone_swarm urdf_test.launch.py
  ```

---

## 5. ROS 2 Interface Summary (Topics, Services & TF Frames)

| Interface | Type | Description |
| :--- | :--- | :--- |
| `/drone_i/cmd_attitude` | `geometry_msgs/msg/Twist` | `linear.z`: Thrust `T` (N); `angular.{x,y,z}`: Desired `(roll_des, pitch_des, yaw_des)` (rad) |
| `/drone_i/joint_states` | `sensor_msgs/msg/JointState` | Continuous joint angles for the 4 propellers (`rotor_fl`, `rotor_fr`, `rotor_bl`, `rotor_br`) |
| `/drone_i/robot_description` | `std_msgs/msg/String` | URDF XML for `drone_i` (`Transient Local` QoS) |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Target 3D centroid and heading for the swarm (set via RViz2 **2D Goal Pose** or CLI) |
| `/swarm_markers` | `visualization_msgs/msg/MarkerArray` | Color-coded spring-damper links, live distance labels (`d_ij`), and golden goal arrow |
| `/swarm_reset` | `std_srvs/srv/Trigger` | Resets all 3 drones to their scattered initial positions |
| `/swarm_disturb` | `std_srvs/srv/Trigger` | Applies a `[1.8, -1.5, 0.8] m/s` velocity impulse to `drone_1` |
| `/tf` & `/tf_static` | `tf2_msgs/msg/TFMessage` | Transform tree:

