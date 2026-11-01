"""Physical, geometric, and formation constants for the quadrotor swarm."""


from __future__ import annotations


import math


# Package and asset paths.
PACKAGE_NAME = 'drone_swarm'
URDF_FILENAME = 'quadrotor.urdf'
SWARM_RVIZ_FILENAME = 'swarm.rviz'
SINGLE_RVIZ_FILENAME = 'urdf_test.rviz'
WORLD_FRAME = 'world'
BASE_LINK_NAME = 'base_link'


# Topics.
GOAL_POSE_TOPIC = '/goal_pose'
SWARM_MARKERS_TOPIC = '/swarm_markers'


# Swarm roster and rotor joint names (generic over any number of drones).
DRONE_NAMES: tuple[str, ...] = ('drone_1', 'drone_2', 'drone_3')
ROTOR_JOINT_NAMES: tuple[str, ...] = (
    'rotor_fl_joint',
    'rotor_fr_joint',
    'rotor_bl_joint',
    'rotor_br_joint',
)


# Physical quadrotor parameters (Proposal Section 5).
MASS_KG = 1.0
GRAVITY_M_S2 = 9.81
HOVER_THRUST_N = MASS_KG * GRAVITY_M_S2
MAX_THRUST_N = 2.5 * HOVER_THRUST_N
MIN_THRUST_Z_N = 0.2 * HOVER_THRUST_N
MIN_ALTITUDE_M = 0.05
LINEAR_DRAG_COEFF = 0.25
INERTIA_DIAG_KG_M2: tuple[float, float, float] = (0.008, 0.008, 0.015)
MAX_TILT_RAD = math.radians(35.0)
BASE_ROTOR_SPIN_RAD_S = 25.0


# Inner-loop attitude PD gains [roll, pitch, yaw].
ATTITUDE_KP: tuple[float, float, float] = (0.8, 0.8, 0.4)
ATTITUDE_KD: tuple[float, float, float] = (0.15, 0.15, 0.10)


# Formation geometry & virtual parallel spring-damper parameters.
TARGET_SEPARATION_M = 2.0
SAFETY_RADIUS_M = 0.6
SAFETY_ACTIVATION_M = 0.95
SAFETY_BARRIER_GAIN = 8.0
SPRING_STIFFNESS_N_M = 3.5
DAMPER_COEFF_N_S_M = 2.8
EQUILIBRIUM_TOLERANCE_M = 0.08


# Outer-loop waypoint & altitude PD gains.
GOAL_KP_XY = 1.5
GOAL_KD_XY = 2.2
ALTITUDE_KP_Z = 4.0
ALTITUDE_KD_Z = 3.2
MAX_HORIZ_ACCEL_M_S2 = 5.0
VELOCITY_FILTER_ALPHA = 0.6


DEFAULT_ALTITUDE_M = 1.0
DEFAULT_GOAL_XYZ: tuple[float, float, float] = (0.0, 0.0, DEFAULT_ALTITUDE_M)
SIM_TIMESTEP_S = 0.02


# Relative equilibrium offsets from formation centroid [dx, dy, dz] (m).
EQUILIBRIUM_OFFSETS: dict[str, tuple[float, float, float]] = {
    'drone_1': (1.1547, 0.0, 0.0),
    'drone_2': (-0.5774, 1.0, 0.0),
    'drone_3': (-0.5774, -1.0, 0.0),
}


# Equilibrium 2.0m equilateral triangle positions in world frame [x, y, z].
EQUILIBRIUM_POSITIONS: dict[str, tuple[float, float, float]] = {
    'drone_1': (1.1547, 0.0, DEFAULT_ALTITUDE_M),
    'drone_2': (-0.5774, 1.0, DEFAULT_ALTITUDE_M),
    'drone_3': (-0.5774, -1.0, DEFAULT_ALTITUDE_M),
}


# Scattered initial positions for dynamic self-assembly demos [x, y, z].
SCATTERED_INITIAL_POSITIONS: dict[str, tuple[float, float, float]] = {
    'drone_1': (1.8, 0.4, 0.6),
    'drone_2': (-1.2, 1.6, 1.4),
    'drone_3': (-0.9, -1.5, 0.9),
}


# Disturbance velocity impulse [vx, vy, vz] (m/s) applied on /swarm_disturb.
DISTURBANCE_VELOCITY_M_S: tuple[float, float, float] = (1.8, -1.5, 0.8)