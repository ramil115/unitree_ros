# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# pytype: disable=attribute-error
"""Real robot interface of A1 robot."""

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


from absl import logging
import math
import re
import numpy as np
import time
import rospy
from motion_imitation.robots import laikago_pose_utils
from motion_imitation.robots import a1
from motion_imitation.robots import a1_robot_velocity_estimator
from motion_imitation.robots import minitaur
from motion_imitation.robots import robot_config
from motion_imitation.envs import locomotion_gym_config
from motion_imitation.robots.A1GazeboInterface import A1GazeboInterface  # pytype: disable=import-error

NUM_MOTORS = 12
NUM_LEGS = 4

HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0.0
KNEE_JOINT_OFFSET = 0.0

JOINT_OFFSETS = np.array(
    [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi

ABDUCTION_P_GAIN = 100.0
ABDUCTION_D_GAIN = 1.0
HIP_P_GAIN = 100.0
HIP_D_GAIN = 2.0
KNEE_P_GAIN = 100.0
KNEE_D_GAIN = 2.0

# Bases on the readings from Laikago's default pose.
INIT_MOTOR_ANGLES = np.array([
    laikago_pose_utils.LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
    laikago_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
    laikago_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE
] * NUM_LEGS)

class A1RobotGazebo(a1.A1):
  """Interface for real A1 robot."""
  def __init__(self, pybullet_client, time_step=0.002,position_control=False,update_rate=100, **kwargs):
    """Initializes the robot class.""" 
    # Initiate UDP for robot state and actions
    self._robot_interface = A1GazeboInterface('a1',position_control,update_rate)
    self.position_control = position_control
    self.motor_kps = np.array([ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN] * 4)
    self.motor_kds = np.array([ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN] * 4)

    self._pybullet_client = pybullet_client
    self.time_step = time_step

    # Robot state variables
    self._init_complete = False
    self._base_orientation = None
    self._raw_state = None
    self._last_raw_state = None
    self._motor_angles = np.zeros(12)
    self._motor_velocities = np.zeros(12)
    self._joint_states = None
    self._last_reset_time = rospy.get_time()
    self._velocity_estimator = a1_robot_velocity_estimator.VelocityEstimator(
        self)

    kwargs['on_rack'] = True
    if not self.position_control:
      super(A1RobotGazebo, self).__init__(pybullet_client,
                                    time_step=time_step,
                                    **kwargs)
                                    
      self._init_complete = True

  def ReceiveObservation(self):
    """Receives observation from robot.

    Synchronous ReceiveObservation is not supported in A1,
    so changging it to noop instead.
    """
    state = self._robot_interface.receive_observation()
    self._raw_state = state
    # Convert quaternion from wxyz to xyzw, which is default for Pybullet.
    q = state.imu.quaternion
    self._base_orientation = np.array([q[1], q[2], q[3], q[0]])
    self._motor_angles = np.array([motor.q for motor in state.motorState[:12]])
    self._motor_velocities = np.array(
        [motor.dq for motor in state.motorState[:12]])
    self._joint_states = np.array(
        list(zip(self._motor_angles, self._motor_velocities)))
    if self._init_complete:
      self._velocity_estimator.update(self._raw_state)

  def GetTrueMotorAngles(self):
    return self._motor_angles.copy()

  def GetMotorAngles(self):
    return minitaur.MapToMinusPiToPi(self._motor_angles).copy()

  def GetMotorVelocities(self):
    return self._motor_velocities.copy()

  def GetBasePosition(self):
    return self._pybullet_client.getBasePositionAndOrientation(
        self.quadruped)[0]

  def GetBaseRollPitchYaw(self):
    if self.position_control:
      return self._pybullet_client.getEulerFromQuaternion(self._robot_interface.getBaseOrientation())
    else:
      return self._pybullet_client.getEulerFromQuaternion(self._base_orientation)

  def GetTrueBaseRollPitchYaw(self):
    return self._pybullet_client.getEulerFromQuaternion(self._base_orientation)

  def GetBaseRollPitchYawRate(self):
    return self.GetTrueBaseRollPitchYawRate()

  def GetTrueBaseRollPitchYawRate(self):
    if self.position_control:
      return self._robot_interface.GetTrueBaseRollPitchYawRate()
    else:
      return np.array(self._raw_state.imu.gyroscope).copy()

  def GetBaseVelocity(self):
    return self._velocity_estimator.estimated_velocity.copy()

  def GetFootContacts(self):
    return np.array(self._raw_state.footForce) > 10.0

  def GetTimeSinceReset(self):
    if self.position_control:
      return self._robot_interface.GetTimeSinceReset()
    else:
      return rospy.get_time() - self._last_reset_time

  def GetBaseOrientation(self):
    return self._base_orientation.copy()

  @property
  def motor_velocities(self):
    return self._motor_velocities.copy()

  def ApplyAction(self, motor_commands, motor_control_mode=None):
    """Clips and then apply the motor commands using the motor model.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).
      motor_control_mode: A MotorControlMode enum.
    """
    if motor_control_mode is None:
      motor_control_mode = self._motor_control_mode

    command = np.zeros(60, dtype=np.float32)
    if motor_control_mode == robot_config.MotorControlMode.POSITION:
      for motor_id in range(NUM_MOTORS):
        command[motor_id * 5] = motor_commands[motor_id]
        command[motor_id * 5 + 1] = self.motor_kps[motor_id]
        command[motor_id * 5 + 3] = self.motor_kds[motor_id]
    elif motor_control_mode == robot_config.MotorControlMode.TORQUE:
      for motor_id in range(NUM_MOTORS):
        command[motor_id * 5 + 4] = motor_commands[motor_id]
    elif motor_control_mode == robot_config.MotorControlMode.HYBRID:
      command = np.array(motor_commands, dtype=np.float32)
    else:
      raise ValueError('Unknown motor control mode for A1 robot: {}.'.format(
          motor_control_mode))
    
    self._robot_interface.send_command(command)

  def Reset(self, reload_urdf=True, default_motor_angles=None, reset_time=3.0):
    """Reset the robot to default motor angles."""
    super(A1RobotGazebo, self).Reset(reload_urdf=reload_urdf,
                               default_motor_angles=default_motor_angles,
                               reset_time=-1)
    logging.warning(
        "About to reset the robot, make sure the robot is hang-up.")

    if not default_motor_angles:
      default_motor_angles = a1.INIT_MOTOR_ANGLES

    current_motor_angles = self.GetMotorAngles()

  
    # Stand up in 1.5 seconds, and keep the behavior in this way.
    standup_time = min(reset_time, 2)
    for t in np.arange(0, reset_time, self.time_step * self._action_repeat):
      blend_ratio = min(t / standup_time, 1)
      action = blend_ratio * default_motor_angles + (
          1 - blend_ratio) * current_motor_angles
      self.Step(action, robot_config.MotorControlMode.POSITION)
      time.sleep(self.time_step * self._action_repeat)
    if self._enable_action_filter:
      self._ResetActionFilter()

    self._velocity_estimator.reset()
    self._state_action_counter = 0
    self._step_counter = 0
    self._last_reset_time = rospy.get_time()

  def Terminate(self):
    self._is_alive = False

  def sendControllerCommand(self, inputCommand):
    if self.position_control:
      return self._robot_interface.sendControllerCommand(inputCommand)
    else:
      return super().sendControllerCommand(inputCommand)

  def _StepInternal(self, action, motor_control_mode=None):
    self.ApplyAction(action, motor_control_mode)
    self.ReceiveObservation()
    self._state_action_counter += 1
