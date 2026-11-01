"""ROS 2 node executing decentralized virtual spring-damper swarm control."""


from __future__ import annotations


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from visualization_msgs.msg import MarkerArray


from drone_swarm import constants
from drone_swarm import formation_control
from drone_swarm import kinematics
from drone_swarm import swarm_markers




class SwarmControllerNode(Node):
  """50 Hz decentralized formation controller reading TF2 and /goal_pose."""


  def __init__(self):
    """Initializes TF2 listener, /goal_pose subscriber, and command pubs."""
    super().__init__('swarm_controller')


    self._tf_buffer = Buffer()
    self._tf_listener = TransformListener(self._tf_buffer, self)


    self._goal_position_m = np.array(constants.DEFAULT_GOAL_XYZ, dtype=float)
    self._goal_yaw_rad = 0.0


    self._prev_positions_m: dict[str, np.ndarray] = {}
    self._velocities_m_s: dict[str, np.ndarray] = {
        name: np.zeros(3, dtype=float) for name in constants.DRONE_NAMES
    }


    self._cmd_pubs = {
        name: self.create_publisher(Twist, f'/{name}/cmd_attitude', 10)
        for name in constants.DRONE_NAMES
    }
    self._marker_pub = self.create_publisher(
        MarkerArray, constants.SWARM_MARKERS_TOPIC, 10
    )
    self._goal_sub = self.create_subscription(
        PoseStamped,
        constants.GOAL_POSE_TOPIC,
        self._on_goal_pose,
        10,
    )


    self._timer = self.create_timer(
        constants.SIM_TIMESTEP_S, self._control_loop
    )
    self.get_logger().info(
        'Swarm Formation Controller started. Use RViz2 "2D Goal Pose" to '
        'fly the triangle formation.'
    )


  def _on_goal_pose(self, msg: PoseStamped) -> None:
    """Updates target swarm centroid [x, y, z] and heading from RViz2."""
    self._goal_position_m[0] = float(msg.pose.position.x)
    self._goal_position_m[1] = float(msg.pose.position.y)
    if msg.pose.position.z > constants.MIN_ALTITUDE_M:
      self._goal_position_m[2] = float(msg.pose.position.z)


    _, _, yaw_rad = kinematics.quaternion_to_euler(
        float(msg.pose.orientation.x),
        float(msg.pose.orientation.y),
        float(msg.pose.orientation.z),
        float(msg.pose.orientation.w),
    )
    self._goal_yaw_rad = yaw_rad
    self.get_logger().info(
        f'New swarm goal centroid: x={self._goal_position_m[0]:.2f}m, '
        f'y={self._goal_position_m[1]:.2f}m, '
        f'z={self._goal_position_m[2]:.2f}m, '
        f'yaw={np.degrees(self._goal_yaw_rad):.1f}deg'
    )


  def _lookup_swarm_states(
      self,
  ) -> dict[str, formation_control.DronePoseVelocity] | None:
    """Queries TF2 for each drone's pose and estimates velocity."""
    swarm_states: dict[str, formation_control.DronePoseVelocity] = {}
    alpha = constants.VELOCITY_FILTER_ALPHA


    for drone_name in constants.DRONE_NAMES:
      target_frame = f'{drone_name}/{constants.BASE_LINK_NAME}'
      try:
        tf_stamped = self._tf_buffer.lookup_transform(
            constants.WORLD_FRAME,
            target_frame,
            Time(),
        )
      except TransformException:
        return None


      trans = tf_stamped.transform.translation
      rot = tf_stamped.transform.rotation
      pos_m = np.array([trans.x, trans.y, trans.z], dtype=float)
      roll_rad, pitch_rad, yaw_rad = kinematics.quaternion_to_euler(
          rot.x, rot.y, rot.z, rot.w
      )
      euler_rad = np.array([roll_rad, pitch_rad, yaw_rad], dtype=float)


      prev_pos = self._prev_positions_m.get(drone_name)
      if prev_pos is not None:
        raw_vel = (pos_m - prev_pos) / constants.SIM_TIMESTEP_S
        self._velocities_m_s[drone_name] = (
            alpha * raw_vel + (1.0 - alpha) * self._velocities_m_s[drone_name]
        )
      self._prev_positions_m[drone_name] = pos_m


      swarm_states[drone_name] = formation_control.DronePoseVelocity(
          position_m=pos_m,
          euler_rad=euler_rad,
          velocity_m_s=self._velocities_m_s[drone_name],
      )


    return swarm_states


  def _control_loop(self) -> None:
    """Computes and publishes attitude commands and spring-damper markers."""
    swarm_states = self._lookup_swarm_states()
    if swarm_states is None:
      return


    for drone_name in constants.DRONE_NAMES:
      thrust_n, desired_euler = formation_control.compute_drone_control_command(
          drone_name,
          swarm_states,
          self._goal_position_m,
          self._goal_yaw_rad,
      )
      cmd_msg = Twist()
      cmd_msg.linear.z = float(thrust_n)
      cmd_msg.angular.x = float(desired_euler[0])
      cmd_msg.angular.y = float(desired_euler[1])
      cmd_msg.angular.z = float(desired_euler[2])
      self._cmd_pubs[drone_name].publish(cmd_msg)


    now_msg = self.get_clock().now().to_msg()
    marker_array = swarm_markers.build_swarm_marker_array(
        swarm_states,
        self._goal_position_m,
        self._goal_yaw_rad,
        now_msg,
    )
    self._marker_pub.publish(marker_array)




def main(args: list[str] | None = None) -> None:
  """Entry point for the swarm_controller ROS 2 executable."""
  rclpy.init(args=args)
  node = SwarmControllerNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
  main()