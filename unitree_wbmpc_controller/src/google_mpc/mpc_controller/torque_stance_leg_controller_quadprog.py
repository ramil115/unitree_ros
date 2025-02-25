# Lint as: python3
"""A torque based stance controller framework."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Sequence, Tuple

import numpy as np
# import time

from mpc_controller import gait_generator as gait_generator_lib
from mpc_controller import leg_controller
from mpc_controller import qp_torque_optimizer

_FORCE_DIMENSION = 3
KP = np.array((100., 100., 100., 100., 100., 100.))
KD = np.array((40., 40., 10., 10., 10., 30.))
MAX_DDQ = np.array((10., 10., 10., 20., 20., 20.))
MIN_DDQ = -MAX_DDQ


class TorqueStanceLegController(leg_controller.LegController):
  """A torque based stance leg controller framework.

  Takes in high level parameters like walking speed and turning speed, and
  generates necessary the torques for stance legs.
  """
  def __init__(
      self,
      robot: Any,
      gait_generator: Any,
      state_estimator: Any,
      desired_speed: Tuple[float, float] = (0, 0),
      desired_twisting_speed: float = 0,
      desired_body_height: float = 0.45,
      num_legs: int = 4,
      friction_coeffs: Sequence[float] = (0.45, 0.45, 0.45, 0.45),
  ):
    """Initializes the class.

    Tracks the desired position/velocity of the robot by computing proper joint
    torques using MPC module.

    Args:
      robot: A robot instance.
      gait_generator: Used to query the locomotion phase and leg states.
      state_estimator: Estimate the robot states (e.g. CoM velocity).
      desired_speed: desired CoM speed in x-y plane.
      desired_twisting_speed: desired CoM rotating speed in z direction.
      desired_body_height: The standing height of the robot.
      body_mass: The total mass of the robot.
      body_inertia: The inertia matrix in the body principle frame. We assume
        the body principle coordinate frame has x-forward and z-up.
      num_legs: The number of legs used for force planning.
      friction_coeffs: The friction coeffs on the contact surfaces.
    """
    self._robot = robot
    self._gait_generator = gait_generator
    self._state_estimator = state_estimator
    self.desired_speed = desired_speed
    self.desired_twisting_speed = desired_twisting_speed

    self._desired_body_height = desired_body_height
    self._num_legs = num_legs
    self._friction_coeffs = np.array(friction_coeffs)

    self._desired_ddq_array = []
    self._contact_forces_array = []

  def reset(self, current_time):
    del current_time

  def update(self, current_time):
    del current_time

  def _estimate_robot_height(self, contacts):
    if np.sum(contacts) == 0:
      # All foot in air, no way to estimate
      return self._desired_body_height
    else:
      base_orientation = self._robot.GetBaseOrientation()
      rot_mat = self._robot.pybullet_client.getMatrixFromQuaternion(
          base_orientation)
      rot_mat = np.array(rot_mat).reshape((3, 3))

      foot_positions = self._robot.GetFootPositionsInBaseFrame()
      foot_positions_world_frame = (rot_mat.dot(foot_positions.T)).T
      # pylint: disable=unsubscriptable-object
      useful_heights = contacts * (-foot_positions_world_frame[:, 2])
      return np.sum(useful_heights) / np.sum(contacts)

  def _estimate_robot_delta_xy(self, contacts):
    if np.sum(contacts) == 0:
      # All foot in air, no way to estimate
      return np.array([0,0])
    else:
      base_orientation = self._robot.GetBaseOrientation()
      rot_mat = self._robot.pybullet_client.getMatrixFromQuaternion(
          base_orientation)
      rot_mat = np.array(rot_mat).reshape((3, 3))

      foot_positions = self._robot.GetFootPositionsInBaseFrame()
      foot_positions_world_frame = (rot_mat.dot(foot_positions.T)).T
      foot_positions_world_frame_0 = np.array([[ 0.171, -0.134, -0.242],
                                               [ 0.171,  0.13 , -0.242],
                                               [-0.182, -0.134, -0.242],
                                               [-0.182,  0.13 , -0.242]])
      diff = foot_positions_world_frame_0 - foot_positions_world_frame
      useful_x = contacts * (diff[:, 0])
      useful_y = contacts * (diff[:, 1])
      return np.array((np.sum(useful_x) / np.sum(contacts), np.sum(useful_y) / np.sum(contacts)))

  def get_action(self):
    """Computes the torque for stance legs."""
    # Actual q and dq
    contacts = np.array(
        [(leg_state in (gait_generator_lib.LegState.STANCE,
                        gait_generator_lib.LegState.EARLY_CONTACT))
         for leg_state in self._gait_generator.desired_leg_state],
        dtype=np.int32)

    robot_com_position = np.array(
        (0., 0., self._estimate_robot_height(contacts)))
    # robot_com_position = np.append(
    #     self._estimate_robot_delta_xy(contacts), self._estimate_robot_height(contacts))
    # print(self._estimate_robot_delta_xy(contacts))
    robot_com_velocity = self._state_estimator.com_velocity_body_frame
    robot_com_roll_pitch_yaw = np.array(self._robot.GetBaseRollPitchYaw())
    robot_com_roll_pitch_yaw[2] = 0  # To prevent yaw drifting
    robot_com_roll_pitch_yaw_rate = self._robot.GetBaseRollPitchYawRate()
    robot_q = np.hstack((robot_com_position, robot_com_roll_pitch_yaw))
    robot_dq = np.hstack((robot_com_velocity, robot_com_roll_pitch_yaw_rate))

    # Desired q and dq
    desired_com_position = np.array((0., 0., self._desired_body_height),
                                    dtype=np.float64)
    desired_com_velocity = np.array(
        (self.desired_speed[0], self.desired_speed[1], 0.), dtype=np.float64)
    desired_com_roll_pitch_yaw = np.array((0., 0., 0.), dtype=np.float64)
    desired_com_angular_velocity = np.array(
        (0., 0., self.desired_twisting_speed), dtype=np.float64)
    desired_q = np.hstack((desired_com_position, desired_com_roll_pitch_yaw))
    desired_dq = np.hstack(
        (desired_com_velocity, desired_com_angular_velocity))
    # Desired ddq
    desired_ddq = KP * (desired_q - robot_q) + KD * (desired_dq - robot_dq)
    desired_ddq = np.clip(desired_ddq, MIN_DDQ, MAX_DDQ)
    self._desired_ddq_array.append(desired_ddq)
    contact_forces = qp_torque_optimizer.compute_contact_force(
        self._robot, desired_ddq, contacts=contacts)
    self._contact_forces_array.append(contact_forces)

    action = {}
    for leg_id, force in enumerate(contact_forces):
      # While "Lose Contact" is useful in simulation, in real environment it's
      # susceptible to sensor noise. Disabling for now.
      # if self._gait_generator.leg_state[
      #     leg_id] == gait_generator_lib.LegState.LOSE_CONTACT:
      #   force = (0, 0, 0)
      motor_torques = self._robot.MapContactForceToJointTorques(leg_id, force)
      for joint_id, torque in motor_torques.items():
        action[joint_id] = (0, 0, 0, 0, torque)
    return action, contact_forces
