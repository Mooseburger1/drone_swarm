"""SO(3), SE(3), velocity Jacobian, and attitude IK for 6-DOF quadrotors."""


from __future__ import annotations


from collections.abc import Sequence
import math


import numpy as np


from drone_swarm import constants


_GIMBAL_LOCK_EPSILON = 1e-4




def wrap_angle_rad(angle_rad: float) -> float:
  """Wraps an angle in radians to the interval [-pi, pi].


  Args:
    angle_rad: Angle in radians.


  Returns:
    Wrapped angle in [-pi, pi].
  """
  return math.atan2(math.sin(angle_rad), math.cos(angle_rad))




def rotation_matrix_zyx(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> np.ndarray:
  """Computes the 3x3 rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll).


  Args:
    roll_rad: Roll angle phi about local X axis in radians.
    pitch_rad: Pitch angle theta about intermediate Y axis in radians.
    yaw_rad: Yaw angle psi about world Z axis in radians.


  Returns:
    A (3, 3) orthogonal rotation matrix in SO(3).
  """
  cos_roll = math.cos(roll_rad)
  sin_roll = math.sin(roll_rad)
  cos_pitch = math.cos(pitch_rad)
  sin_pitch = math.sin(pitch_rad)
  cos_yaw = math.cos(yaw_rad)
  sin_yaw = math.sin(yaw_rad)


  return np.array(
      [
          [
              cos_yaw * cos_pitch,
              cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
              cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll,
          ],
          [
              sin_yaw * cos_pitch,
              sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
              sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll,
          ],
          [
              -sin_pitch,
              cos_pitch * sin_roll,
              cos_pitch * cos_roll,
          ],
      ],
      dtype=float,
  )




def homogeneous_transform(
    position_xyz: Sequence[float],
    euler_rpy_rad: Sequence[float],
) -> np.ndarray:
  """Constructs a 4x4 homogeneous transformation matrix T in SE(3).


  Args:
    position_xyz: 3D translation vector [x, y, z] in meters.
    euler_rpy_rad: ZYX Euler angles [roll, pitch, yaw] in radians.


  Returns:
    A (4, 4) homogeneous transformation matrix T_body^world.
  """
  transform = np.eye(4, dtype=float)
  transform[:3, :3] = rotation_matrix_zyx(
      euler_rpy_rad[0],
      euler_rpy_rad[1],
      euler_rpy_rad[2],
  )
  transform[:3, 3] = np.asarray(position_xyz, dtype=float)
  return transform




def invert_homogeneous_transform(transform: np.ndarray) -> np.ndarray:
  """Inverts a 4x4 rigid-body homogeneous transformation matrix in SE(3).


  Uses the analytic transpose property R^T to avoid numerical matrix inversion.


  Args:
    transform: A (4, 4) homogeneous transformation matrix.


  Returns:
    The (4, 4) inverse transformation matrix T^-1.
  """
  rotation_transpose = transform[:3, :3].T
  translation = transform[:3, 3]
  inv_transform = np.eye(4, dtype=float)
  inv_transform[:3, :3] = rotation_transpose
  inv_transform[:3, 3] = -(rotation_transpose @ translation)
  return inv_transform




def relative_transform(
    transform_i_world: np.ndarray,
    transform_j_world: np.ndarray,
) -> np.ndarray:
  """Computes neighbor j's pose in drone i's body frame: T_j^i = (T_i^0)^-1 T_j^0.


  Args:
    transform_i_world: (4, 4) homogeneous transform of drone i in world frame.
    transform_j_world: (4, 4) homogeneous transform of drone j in world frame.


  Returns:
    A (4, 4) homogeneous transformation matrix T_j^i.
  """
  return invert_homogeneous_transform(transform_i_world) @ transform_j_world




def euler_rate_jacobian(roll_rad: float, pitch_rad: float) -> np.ndarray:
  """Computes the rotational velocity Jacobian W(roll, pitch).


  Maps body-frame angular velocities [p, q, r]^T to ZYX Euler angle rates
  [roll_dot, pitch_dot, yaw_dot]^T. Guards against the pitch = +/- pi/2
  gimbal-lock singularity.


  Args:
    roll_rad: Roll angle phi in radians.
    pitch_rad: Pitch angle theta in radians.


  Returns:
    A (3, 3) Jacobian matrix W.
  """
  cos_roll = math.cos(roll_rad)
  sin_roll = math.sin(roll_rad)
  cos_pitch = math.cos(pitch_rad)
  tan_pitch = math.tan(pitch_rad)


  if abs(cos_pitch) < _GIMBAL_LOCK_EPSILON:
    cos_pitch = math.copysign(_GIMBAL_LOCK_EPSILON, cos_pitch)


  return np.array(
      [
          [1.0, sin_roll * tan_pitch, cos_roll * tan_pitch],
          [0.0, cos_roll, -sin_roll],
          [0.0, sin_roll / cos_pitch, cos_roll / cos_pitch],
      ],
      dtype=float,
  )




def euler_to_quaternion(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
  """Converts ZYX Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).


  Args:
    roll_rad: Roll angle in radians.
    pitch_rad: Pitch angle in radians.
    yaw_rad: Yaw angle in radians.


  Returns:
    Tuple of (qx, qy, qz, qw).
  """
  cos_half_r = math.cos(roll_rad * 0.5)
  sin_half_r = math.sin(roll_rad * 0.5)
  cos_half_p = math.cos(pitch_rad * 0.5)
  sin_half_p = math.sin(pitch_rad * 0.5)
  cos_half_y = math.cos(yaw_rad * 0.5)
  sin_half_y = math.sin(yaw_rad * 0.5)


  qx = sin_half_r * cos_half_p * cos_half_y - (
      cos_half_r * sin_half_p * sin_half_y
  )
  qy = cos_half_r * sin_half_p * cos_half_y + (
      sin_half_r * cos_half_p * sin_half_y
  )
  qz = cos_half_r * cos_half_p * sin_half_y - (
      sin_half_r * sin_half_p * cos_half_y
  )
  qw = cos_half_r * cos_half_p * cos_half_y + (
      sin_half_r * sin_half_p * sin_half_y
  )
  return qx, qy, qz, qw




def quaternion_to_euler(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[float, float, float]:
  """Converts a unit quaternion (x, y, z, w) to ZYX Euler angles (rad).


  Args:
    qx: Quaternion X component.
    qy: Quaternion Y component.
    qz: Quaternion Z component.
    qw: Quaternion W scalar component.


  Returns:
    Tuple of (roll_rad, pitch_rad, yaw_rad).
  """
  sinr_cosp = 2.0 * (qw * qx + qy * qz)
  cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
  roll_rad = math.atan2(sinr_cosp, cosr_cosp)


  sinp = 2.0 * (qw * qy - qz * qx)
  sinp_clamped = max(-1.0, min(1.0, sinp))
  pitch_rad = math.asin(sinp_clamped)


  siny_cosp = 2.0 * (qw * qz + qx * qy)
  cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
  yaw_rad = math.atan2(siny_cosp, cosy_cosp)
  return roll_rad, pitch_rad, yaw_rad




def compute_attitude_ik(
    desired_accel_world: Sequence[float],
    current_yaw_rad: float,
    mass_kg: float = constants.MASS_KG,
    gravity_m_s2: float = constants.GRAVITY_M_S2,
    max_tilt_rad: float = constants.MAX_TILT_RAD,
) -> tuple[float, float, float]:
  """Solves 6-DOF underactuated attitude inverse kinematics.


  Inverts a desired 3D acceleration vector in the world frame (plus gravity
  compensation) into total upward thrust T and desired roll and pitch angles.


  Args:
    desired_accel_world: Desired [ax, ay, az] acceleration in world frame.
    current_yaw_rad: Current yaw angle psi of the quadrotor in radians.
    mass_kg: Quadrotor mass in kg.
    gravity_m_s2: Gravitational acceleration in m/s^2.
    max_tilt_rad: Maximum allowable roll/pitch angle in radians.


  Returns:
    Tuple of (thrust_n, desired_roll_rad, desired_pitch_rad).
  """
  ax, ay, az = (
      float(desired_accel_world[0]),
      float(desired_accel_world[1]),
      float(desired_accel_world[2]),
  )
  force_req_world = np.array(
      [
          mass_kg * ax,
          mass_kg * ay,
          max(mass_kg * (az + gravity_m_s2), constants.MIN_THRUST_Z_N),
      ],
      dtype=float,
  )
  thrust_n = float(
      np.clip(np.linalg.norm(force_req_world), 0.0, constants.MAX_THRUST_N)
  )


  # Rotate horizontal force components into the yaw-aligned frame.
  cos_yaw = math.cos(current_yaw_rad)
  sin_yaw = math.sin(current_yaw_rad)
  fx_yaw = cos_yaw * force_req_world[0] + sin_yaw * force_req_world[1]
  fy_yaw = -sin_yaw * force_req_world[0] + cos_yaw * force_req_world[1]
  fz = force_req_world[2]


  desired_roll_rad = math.atan2(-fy_yaw, math.sqrt(fx_yaw * fx_yaw + fz * fz))
  desired_pitch_rad = math.atan2(fx_yaw, fz)


  desired_roll_rad = float(
      np.clip(desired_roll_rad, -max_tilt_rad, max_tilt_rad)
  )
  desired_pitch_rad = float(
      np.clip(desired_pitch_rad, -max_tilt_rad, max_tilt_rad)
  )
  return thrust_n, desired_roll_rad, desired_pitch_rad