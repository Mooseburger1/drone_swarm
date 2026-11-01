"""6-DOF rigid-body state and Newton-Euler dynamics for a quadrotor."""


from __future__ import annotations


from collections.abc import Sequence
import dataclasses
import math


import numpy as np


from drone_swarm import constants
from drone_swarm import kinematics




@dataclasses.dataclass
class QuadrotorParams:
  """Physical parameters of a single 6-DOF quadrotor."""
  mass_kg: float = constants.MASS_KG
  gravity_m_s2: float = constants.GRAVITY_M_S2
  linear_drag_coeff: float = constants.LINEAR_DRAG_COEFF
  max_tilt_rad: float = constants.MAX_TILT_RAD
  max_thrust_n: float = constants.MAX_THRUST_N
  min_altitude_m: float = constants.MIN_ALTITUDE_M
  base_rotor_spin_rad_s: float = constants.BASE_ROTOR_SPIN_RAD_S
  inertia_diag: tuple[float, float, float] = constants.INERTIA_DIAG_KG_M2
  attitude_kp: tuple[float, float, float] = constants.ATTITUDE_KP
  attitude_kd: tuple[float, float, float] = constants.ATTITUDE_KD




class QuadrotorState:
  """Maintains the 6-DOF kinematic and dynamic state of one quadrotor."""
  def __init__(
      self,
      name: str,
      initial_position_xyz: Sequence[float],
      hover_thrust_n: float = constants.HOVER_THRUST_N,
  ):
    """Initializes the quadrotor state at rest at initial_position_xyz.


    Args:
      name: Unique identifier for the drone (e.g. 'drone_1').
      initial_position_xyz: Starting [x, y, z] coordinates in world frame (m).
      hover_thrust_n: Default upward thrust in Newtons.
    """
    self.name = name
    self.position_m = np.array(initial_position_xyz, dtype=float)
    self.velocity_m_s = np.zeros(3, dtype=float)
    self.euler_rad = np.zeros(3, dtype=float)
    self.omega_body_rad_s = np.zeros(3, dtype=float)
    self.rotor_angles_rad = np.zeros(4, dtype=float)


    self.cmd_thrust_n = float(hover_thrust_n)
    self.cmd_euler_rad = np.zeros(3, dtype=float)




def step_quadrotor_dynamics(
    drone_state: QuadrotorState,
    params: QuadrotorParams,
    dt_s: float,
) -> None:
  """Advances a single quadrotor's 6-DOF state by one timestep dt_s in place.


  Args:
    drone_state: Mutable state of the quadrotor to advance.
    params: Physical and controller parameters of the quadrotor.
    dt_s: Simulation integration step in seconds.
  """
  roll_rad, pitch_rad, yaw_rad = drone_state.euler_rad


  # 1. Rotation matrix R_i^0 in SO(3).
  rot_body_to_world = kinematics.rotation_matrix_zyx(
      roll_rad, pitch_rad, yaw_rad
  )


  # 2. Translational Newton-Euler dynamics in world frame:
  #    m * a = R * [0, 0, T]^T - [0, 0, m*g]^T - c_drag * v
  thrust_world = rot_body_to_world @ np.array(
      [0.0, 0.0, drone_state.cmd_thrust_n]
  )
  gravity_world = np.array([0.0, 0.0, params.mass_kg * params.gravity_m_s2])
  drag_world = params.linear_drag_coeff * drone_state.velocity_m_s
  accel_world = (thrust_world - gravity_world - drag_world) / params.mass_kg


  drone_state.velocity_m_s += accel_world * dt_s
  drone_state.position_m += drone_state.velocity_m_s * dt_s


  if drone_state.position_m[2] < params.min_altitude_m:
    drone_state.position_m[2] = params.min_altitude_m
    if drone_state.velocity_m_s[2] < 0.0:
      drone_state.velocity_m_s[2] = 0.0


  # 3. Inner-loop attitude PD control & Rotational Euler dynamics:
  #    I * omega_dot + omega x (I * omega) = tau
  att_error = drone_state.cmd_euler_rad - drone_state.euler_rad
  att_error[2] = kinematics.wrap_angle_rad(float(att_error[2]))


  kp_vec = np.asarray(params.attitude_kp, dtype=float)
  kd_vec = np.asarray(params.attitude_kd, dtype=float)
  torque_body = kp_vec * att_error - kd_vec * drone_state.omega_body_rad_s


  inertia_vec = np.asarray(params.inertia_diag, dtype=float)
  coriolis_torque = np.cross(
      drone_state.omega_body_rad_s,
      inertia_vec * drone_state.omega_body_rad_s,
  )
  omega_dot = (torque_body - coriolis_torque) / inertia_vec
  drone_state.omega_body_rad_s += omega_dot * dt_s


  # 4. Rotational velocity Jacobian: euler_dot = W(roll, pitch) * omega.
  jacobian_w = kinematics.euler_rate_jacobian(roll_rad, pitch_rad)
  euler_dot = jacobian_w @ drone_state.omega_body_rad_s
  drone_state.euler_rad += euler_dot * dt_s
  drone_state.euler_rad[0] = float(
      np.clip(
          drone_state.euler_rad[0], -params.max_tilt_rad, params.max_tilt_rad
      )
  )
  drone_state.euler_rad[1] = float(
      np.clip(
          drone_state.euler_rad[1], -params.max_tilt_rad, params.max_tilt_rad
      )
  )
  drone_state.euler_rad[2] = kinematics.wrap_angle_rad(
      float(drone_state.euler_rad[2])
  )


  # 5. Update continuous propeller joint angles proportional to thrust.
  hover_thrust = params.mass_kg * params.gravity_m_s2
  thrust_ratio = max(drone_state.cmd_thrust_n, 0.0) / hover_thrust
  spin_rate = params.base_rotor_spin_rad_s * math.sqrt(thrust_ratio)
  rotor_rates = np.array([spin_rate, -spin_rate, -spin_rate, spin_rate])
  drone_state.rotor_angles_rad = (
      drone_state.rotor_angles_rad + rotor_rates * dt_s
  ) % (2.0 * math.pi)