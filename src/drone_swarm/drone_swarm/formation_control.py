"""Decentralized parallel virtual spring-damper formation control law."""


from __future__ import annotations


from collections.abc import Mapping
import dataclasses
import math


import numpy as np


from drone_swarm import constants
from drone_swarm import kinematics




@dataclasses.dataclass
class DronePoseVelocity:
  """Observed 6-DOF pose and linear velocity of a drone in the world frame."""


  position_m: np.ndarray
  euler_rad: np.ndarray
  velocity_m_s: np.ndarray




def compute_spring_damper_force_world(
    drone_name: str,
    swarm_states: Mapping[str, DronePoseVelocity],
) -> np.ndarray:
  """Computes the net parallel virtual spring-damper force on drone_name.


  Uses SE(3) homogeneous transformations T_j^i = (T_i^0)^-1 * T_j^0 to compute
  each neighbor j's relative position in drone i's local body frame before
  mapping the resulting force vector into the world frame.


  Args:
    drone_name: Name of the controlled drone i.
    swarm_states: Mapping of all drone names to their observed pose & velocity.


  Returns:
    Net 3D virtual spring-damper and safety barrier force vector in Newtons.
  """
  state_i = swarm_states[drone_name]
  transform_i_world = kinematics.homogeneous_transform(
      state_i.position_m, state_i.euler_rad
  )
  rot_i_world = transform_i_world[:3, :3]


  net_force_world = np.zeros(3, dtype=float)


  for neighbor_name, state_j in swarm_states.items():
    if neighbor_name == drone_name:
      continue


    # 1. Compute relative transform in drone i's tilted body frame:
    #    T_j^i = (T_i^0)^-1 * T_j^0
    transform_j_world = kinematics.homogeneous_transform(
        state_j.position_m, state_j.euler_rad
    )
    transform_j_in_i = kinematics.relative_transform(
        transform_i_world, transform_j_world
    )
    rel_pos_body_i = transform_j_in_i[:3, 3]


    # Map relative vector from body frame i to world frame: r_ij = R_i^0 * p_ij^i.
    rel_pos_world = rot_i_world @ rel_pos_body_i
    distance_m = float(np.linalg.norm(rel_pos_world))
    unit_los = rel_pos_world / max(distance_m, 1e-6)


    # 2. Formation rigidity Jacobian distance rate: d_dot_ij = u_ij^T (v_j - v_i).
    rel_vel_world = state_j.velocity_m_s - state_i.velocity_m_s
    distance_rate_m_s = float(np.dot(unit_los, rel_vel_world))


    # 3. Parallel Kelvin-Voigt spring-damper force magnitude.
    distance_error_m = distance_m - constants.TARGET_SEPARATION_M
    force_mag_n = (
        constants.SPRING_STIFFNESS_N_M * distance_error_m
        + constants.DAMPER_COEFF_N_S_M * distance_rate_m_s
    )


    # 4. Short-range repulsive safety bubble barrier (d_min = 0.6 m).
    if distance_m < constants.SAFETY_ACTIVATION_M:
      clearance_m = max(distance_m - constants.SAFETY_RADIUS_M, 0.05)
      penetration_m = constants.SAFETY_ACTIVATION_M - distance_m
      force_mag_n -= constants.SAFETY_BARRIER_GAIN * (
          penetration_m / clearance_m
      )


    net_force_world += force_mag_n * unit_los


  return net_force_world




def compute_drone_control_command(
    drone_name: str,
    swarm_states: Mapping[str, DronePoseVelocity],
    goal_position_m: np.ndarray,
    goal_yaw_rad: float,
) -> tuple[float, tuple[float, float, float]]:
  """Computes the attitude & thrust command for a single drone in the swarm.


  Args:
    drone_name: Name of the controlled drone.
    swarm_states: Mapping of all drone names to their observed pose & velocity.
    goal_position_m: Target 3D centroid [x, y, z] of the swarm in meters.
    goal_yaw_rad: Target formation heading yaw angle in radians.


  Returns:
    Tuple of (thrust_n, (desired_roll_rad, desired_pitch_rad, desired_yaw_rad)).
  """
  state_i = swarm_states[drone_name]
  spring_force_world = compute_spring_damper_force_world(
      drone_name, swarm_states
  )


  # Rotate formation slot offset by goal_yaw_rad around the target centroid.
  cos_yaw = math.cos(goal_yaw_rad)
  sin_yaw = math.sin(goal_yaw_rad)
  rot_z = np.array(
      [
          [cos_yaw, -sin_yaw, 0.0],
          [sin_yaw, cos_yaw, 0.0],
          [0.0, 0.0, 1.0],
      ],
      dtype=float,
  )
  slot_offset = np.asarray(
      constants.EQUILIBRIUM_OFFSETS[drone_name], dtype=float
  )
  target_pos_m = goal_position_m + rot_z @ slot_offset


  pos_error_m = target_pos_m - state_i.position_m
  nav_force_world = (
      constants.GOAL_KP_XY * pos_error_m -
constants.GOAL_KD_XY * state_i.velocity_m_s
  )
  nav_force_world[2] = (
      constants.ALTITUDE_KP_Z * (goal_position_m[2] - state_i.position_m[2]) -
constants.ALTITUDE_KD_Z * state_i.velocity_m_s[2]
  )


  desired_accel_world = (
      spring_force_world + nav_force_world
  ) / constants.MASS_KG
  horiz_accel_norm = float(np.linalg.norm(desired_accel_world[:2]))
  if horiz_accel_norm > constants.MAX_HORIZ_ACCEL_M_S2:
    desired_accel_world[:2] *= (
        constants.MAX_HORIZ_ACCEL_M_S2 / horiz_accel_norm
    )


  thrust_n, roll_des_rad, pitch_des_rad = kinematics.compute_attitude_ik(
      desired_accel_world,
      float(state_i.euler_rad[2]),
  )
  return thrust_n, (roll_des_rad, pitch_des_rad, goal_yaw_rad)