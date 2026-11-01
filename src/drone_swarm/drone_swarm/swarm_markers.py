"""RViz2 MarkerArray builders for visualizing virtual spring-damper links."""


from __future__ import annotations


from collections.abc import Mapping
import itertools


from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


from drone_swarm import constants
from drone_swarm import formation_control
from drone_swarm import kinematics




def _spring_color(distance_m: float) -> ColorRGBA:
  """Selects green at equilibrium, red when compressed, blue when stretched."""
  error_m = distance_m - constants.TARGET_SEPARATION_M
  color = ColorRGBA(a=0.9)
  if abs(error_m) <= constants.EQUILIBRIUM_TOLERANCE_M:
    color.r, color.g, color.b = 0.1, 0.9, 0.25
  elif error_m < 0.0:
    color.r, color.g, color.b = 0.95, 0.2, 0.15
  else:
    color.r, color.g, color.b = 0.15, 0.55, 1.0
  return color




def build_swarm_marker_array(
    swarm_states: Mapping[str, formation_control.DronePoseVelocity],
    goal_position_m: np.ndarray,
    goal_yaw_rad: float,
    stamp_msg,
) -> MarkerArray:
  """Builds a MarkerArray of spring-damper links, distance labels, and goal.


  Args:
    swarm_states: Mapping of drone names to their observed pose & velocity.
    goal_position_m: Current 3D target centroid [x, y, z] in meters.
    goal_yaw_rad: Current target formation yaw angle in radians.
    stamp_msg: ROS Time message for marker headers.


  Returns:
    Populated MarkerArray message for RViz2.
  """
  marker_array = MarkerArray()
  drone_names = sorted(swarm_states.keys())


  for pair_idx, (name_i, name_j) in enumerate(
      itertools.combinations(drone_names, 2)
  ):
    pos_i = swarm_states[name_i].position_m
    pos_j = swarm_states[name_j].position_m
    distance_m = float(np.linalg.norm(pos_j - pos_i))


    # 1. Spring-damper line connecting drone i and drone j.
    line_marker = Marker()
    line_marker.header.stamp = stamp_msg
    line_marker.header.frame_id = constants.WORLD_FRAME
    line_marker.ns = 'spring_links'
    line_marker.id = pair_idx
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.035
    line_marker.color = _spring_color(distance_m)
    line_marker.points = [
        Point(x=float(pos_i[0]), y=float(pos_i[1]), z=float(pos_i[2])),
        Point(x=float(pos_j[0]), y=float(pos_j[1]), z=float(pos_j[2])),
    ]
    marker_array.markers.append(line_marker)


    # 2. Floating distance text label at the midpoint of the link.
    midpoint = 0.5 * (pos_i + pos_j)
    text_marker = Marker()
    text_marker.header.stamp = stamp_msg
    text_marker.header.frame_id = constants.WORLD_FRAME
    text_marker.ns = 'spring_labels'
    text_marker.id = pair_idx + 100
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = float(midpoint[0])
    text_marker.pose.position.y = float(midpoint[1])
    text_marker.pose.position.z = float(midpoint[2]) + 0.14
    text_marker.scale.z = 0.14
    text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
    idx_i = name_i.split('_')[-1]
    idx_j = name_j.split('_')[-1]
    text_marker.text = f'd_{idx_i}{idx_j}: {distance_m:.2f}m'
    marker_array.markers.append(text_marker)


  # 3. Goal centroid arrow marker.
  goal_marker = Marker()
  goal_marker.header.stamp = stamp_msg
  goal_marker.header.frame_id = constants.WORLD_FRAME
  goal_marker.ns = 'swarm_goal'
  goal_marker.id = 999
  goal_marker.type = Marker.ARROW
  goal_marker.action = Marker.ADD
  goal_marker.pose.position.x = float(goal_position_m[0])
  goal_marker.pose.position.y = float(goal_position_m[1])
  goal_marker.pose.position.z = float(goal_position_m[2])
  qx, qy, qz, qw = kinematics.euler_to_quaternion(0.0, 0.0, goal_yaw_rad)
  goal_marker.pose.orientation.x = qx
  goal_marker.pose.orientation.y = qy
  goal_marker.pose.orientation.z = qz
  goal_marker.pose.orientation.w = qw
  goal_marker.scale.x = 0.45
  goal_marker.scale.y = 0.08
  goal_marker.scale.z = 0.08
  goal_marker.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.9)
  marker_array.markers.append(goal_marker)


  return marker_array