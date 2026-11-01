"""ROS 2 node wrapping the 6-DOF quadrotor swarm kinematics and dynamics."""


from __future__ import annotations


import functools


from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


from drone_swarm import constants
from drone_swarm import dynamics
from drone_swarm import kinematics




class SwarmSimulatorNode(Node):
  """Simulates a 6-DOF quadrotor swarm at 50 Hz and broadcasts TF/JointStates."""


  def __init__(self):
    """Initializes publishers, subscribers, services, and simulation timer."""
    super().__init__('swarm_sim')


    self._params = dynamics.QuadrotorParams()
    self._drones: dict[str, dynamics.QuadrotorState] = {}
    self._reset_drone_states()


    self._tf_broadcaster = TransformBroadcaster(self)
    self._joint_pubs = {}
    self._cmd_subs = {}


    for drone_name in constants.DRONE_NAMES:
      self._joint_pubs[drone_name] = self.create_publisher(
          JointState,
          f'/{drone_name}/joint_states',
          10,
      )
      self._cmd_subs[drone_name] = self.create_subscription(
          Twist,
          f'/{drone_name}/cmd_attitude',
          functools.partial(self._on_attitude_command, drone_name=drone_name),
          10,
      )


    self._reset_srv = self.create_service(
        Trigger, '/swarm_reset', self._handle_reset
    )
    self._disturb_srv = self.create_service(
        Trigger, '/swarm_disturb', self._handle_disturb
    )


    self._timer = self.create_timer(
        constants.SIM_TIMESTEP_S, self._step_simulation
    )
    drone_roster = ', '.join(constants.DRONE_NAMES)
    self.get_logger().info(
        f'6-DOF Swarm Simulator started for {drone_roster}.'
    )


  def _reset_drone_states(self) -> None:
    """Resets all drones to their default scattered initial positions."""
    for drone_name in constants.DRONE_NAMES:
      init_xyz = constants.SCATTERED_INITIAL_POSITIONS[drone_name]
      self._drones[drone_name] = dynamics.QuadrotorState(drone_name, init_xyz)


  def _on_attitude_command(self, msg: Twist, drone_name: str) -> None:
    """Updates target thrust and clamped roll/pitch/yaw for a single drone."""
    drone_state = self._drones[drone_name]
    max_tilt = self._params.max_tilt_rad
    drone_state.cmd_thrust_n = float(
        np.clip(msg.linear.z, 0.0, self._params.max_thrust_n)
    )
    drone_state.cmd_euler_rad[0] = float(
        np.clip(msg.angular.x, -max_tilt, max_tilt)
    )
    drone_state.cmd_euler_rad[1] = float(
        np.clip(msg.angular.y, -max_tilt, max_tilt)
    )
    drone_state.cmd_euler_rad[2] = float(msg.angular.z)


  def _handle_reset(
      self,
      unused_request: Trigger.Request,
      response: Trigger.Response,
  ) -> Trigger.Response:
    """Handles /swarm_reset service requests."""
    self._reset_drone_states()
    response.success = True
    response.message = 'Swarm reset to scattered initial positions.'
    self.get_logger().info(response.message)
    return response


  def _handle_disturb(
      self,
      unused_request: Trigger.Request,
      response: Trigger.Response,
  ) -> Trigger.Response:
    """Applies a velocity disturbance impulse to the first drone."""
    target_drone = constants.DRONE_NAMES[0]
    impulse = np.asarray(constants.DISTURBANCE_VELOCITY_M_S, dtype=float)
    self._drones[target_drone].velocity_m_s += impulse
    response.success = True
    response.message = (
        f'Applied velocity impulse {constants.DISTURBANCE_VELOCITY_M_S} m/s '
        f'to {target_drone}.'
    )
    self.get_logger().info(response.message)
    return response


  def _step_simulation(self) -> None:
    """Integrates one physics step for all drones and publishes TF/joints."""
    now_msg = self.get_clock().now().to_msg()
    transform_msgs: list[TransformStamped] = []


    for drone_name, drone_state in self._drones.items():
      dynamics.step_quadrotor_dynamics(
          drone_state, self._params, constants.SIM_TIMESTEP_S
      )
      transform_msgs.append(self._build_transform_msg(drone_state, now_msg))
      self._joint_pubs[drone_name].publish(
          self._build_joint_state_msg(drone_state, now_msg)
      )


    self._tf_broadcaster.sendTransform(transform_msgs)


  def _build_transform_msg(
      self,
      drone_state: dynamics.QuadrotorState,
      stamp_msg,
  ) -> TransformStamped:
    """Builds a TransformStamped from world to drone_i/base_link."""
    transform_msg = TransformStamped()
    transform_msg.header.stamp = stamp_msg
    transform_msg.header.frame_id = constants.WORLD_FRAME
    transform_msg.child_frame_id = (
        f'{drone_state.name}/{constants.BASE_LINK_NAME}'
    )
    transform_msg.transform.translation.x = float(drone_state.position_m[0])
    transform_msg.transform.translation.y = float(drone_state.position_m[1])
    transform_msg.transform.translation.z = float(drone_state.position_m[2])


    qx, qy, qz, qw = kinematics.euler_to_quaternion(
        float(drone_state.euler_rad[0]),
        float(drone_state.euler_rad[1]),
        float(drone_state.euler_rad[2]),
    )
    transform_msg.transform.rotation.x = qx
    transform_msg.transform.rotation.y = qy
    transform_msg.transform.rotation.z = qz
    transform_msg.transform.rotation.w = qw
    return transform_msg


  def _build_joint_state_msg(
      self,
      drone_state: dynamics.QuadrotorState,
      stamp_msg,
  ) -> JointState:
    """Builds a JointState message for a drone's 4 spinning rotors."""
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = stamp_msg
    joint_state_msg.name = list(constants.ROTOR_JOINT_NAMES)
    joint_state_msg.position = [
        float(angle) for angle in drone_state.rotor_angles_rad
    ]
    return joint_state_msg




def main(args: list[str] | None = None) -> None:
  """Entry point for the swarm_sim ROS 2 executable."""
  rclpy.init(args=args)
  node = SwarmSimulatorNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    # Expected on Ctrl-C shutdown from terminal or ros2 launch.
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
  main()