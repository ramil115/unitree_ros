"""Example of whole body controller on A1 robot."""
import os
import inspect

from numpy.core.numeric import roll
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import shlex
from psutil import Popen

from absl import app
from absl import flags
from absl import logging
from datetime import datetime
import numpy as np
import os
import scipy.interpolate
import time
import matplotlib.pyplot as plt

import pybullet_data
from pybullet_utils import bullet_client
import pybullet  # pytype:disable=import-error

from mpc_controller import com_velocity_estimator
from mpc_controller import gait_generator as gait_generator_lib
from mpc_controller import locomotion_controller
from mpc_controller import openloop_gait_generator
from mpc_controller import raibert_swing_leg_controller
# from mpc_controller import torque_stance_leg_controller
#import mpc_osqp
from mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller


from motion_imitation.robots import a1
from motion_imitation.robots import robot_config
from motion_imitation.robots.gamepad import gamepad_reader

# flags.DEFINE_string("logdir", "log_data", "where to log trajectories.")
flags.DEFINE_string("logdir", None, "where to log trajectories.")
flags.DEFINE_bool("use_gamepad", False,
                  "whether to use gamepad to provide control input.")
flags.DEFINE_bool("use_gazebo", False,
                  "whether to use real robot or simulation")
flags.DEFINE_bool("use_real_robot", True,
                  "whether to use real robot or simulation")
flags.DEFINE_bool("show_gui", True, "whether to show GUI.")
flags.DEFINE_float("max_time_secs", 20., "maximum time to run the robot.")
flags.DEFINE_bool("pos_control", False, "use positional control?")
FLAGS = flags.FLAGS

_NUM_SIMULATION_ITERATION_STEPS = 300
_MAX_TIME_SECONDS = 30.

_STANCE_DURATION_SECONDS = [
    0.3
] * 4  # For faster trotting (v > 1.5 ms reduce this to 0.13s).

# Standing
# _DUTY_FACTOR = [1.] * 4
# _INIT_PHASE_FULL_CYCLE = [0., 0., 0., 0.]

# _INIT_LEG_STATE = (
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
# )

# Tripod
# _DUTY_FACTOR = [.8] * 4
# _INIT_PHASE_FULL_CYCLE = [0., 0.25, 0.5, 0.]

# _INIT_LEG_STATE = (
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.SWING,
# )

# Trotting
_DUTY_FACTOR = [0.6] * 4
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]

_INIT_LEG_STATE = (
    gait_generator_lib.LegState.SWING,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.SWING,
)


def _generate_example_linear_angular_speed(t):
  """Creates an example speed profile based on time for demo purpose."""
  vx = 0.8
  vy = 0.2
  wz = 0.8
  # vx = 0.0
  # vy = 0.0
  # wz = 0.0

  # time_points = (0, 5, 10, 15, 20, 25, 30)
  # speed_points = ((0, 0, 0, 0), (vx, 0, 0, 0), (vx, 0, 0, 0), (0, 0, 0, -wz),
  #                 (0, -vy, 0, 0), (0, 0, 0, 0), (0, 0, 0, wz))

  time_points = (0, 20)
  speed_points = ((0, 0, 0, 0), (vx, 0, 0, 0))

  speed = scipy.interpolate.interp1d(time_points,
                                     speed_points,
                                     kind="linear", # pay attention to kind (default "linear")
                                     fill_value="extrapolate",
                                     axis=0)(t)

  return speed[0:3], speed[3], False


def _setup_controller(robot):
  """Demonstrates how to create a locomotion controller."""
  desired_speed = (0, 0)
  desired_twisting_speed = 0

  gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
      robot,
      stance_duration=_STANCE_DURATION_SECONDS,
      duty_factor=_DUTY_FACTOR,
      initial_leg_phase=_INIT_PHASE_FULL_CYCLE,
      initial_leg_state=_INIT_LEG_STATE)
  window_size = 20 if not FLAGS.use_real_robot else 1
  state_estimator = com_velocity_estimator.COMVelocityEstimator(
      robot, window_size=window_size)
  sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_height=robot.MPC_BODY_HEIGHT,
      foot_clearance=0.01)

  st_controller = torque_stance_leg_controller.TorqueStanceLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_body_height=robot.MPC_BODY_HEIGHT
      #,qp_solver = mpc_osqp.QPOASES #or mpc_osqp.OSQP
      )

  controller = locomotion_controller.LocomotionController(
      robot=robot,
      gait_generator=gait_generator,
      state_estimator=state_estimator,
      swing_leg_controller=sw_controller,
      stance_leg_controller=st_controller,
      clock=robot.GetTimeSinceReset)

  return controller

from gazebo_msgs.msg import  ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Vector3
import rospy
from std_msgs.msg import Float64

class InputCommand:
  def __init__(self, linearSpeed,angularSpeed):
      self.linearSpeed = linearSpeed
      self.angularSpeed = angularSpeed
      self.useIMU = True
  
  def transformToPosControl(self):
    inputVec = [0,0,1,0,0,1,0,0]
    buttons = [False]*8

    inputVec[4] = self.linearSpeed[0]
    inputVec[3] = self.linearSpeed[1]
    inputVec[0] = self.angularSpeed

    if self.useIMU == True:
      buttons[7]=True

    return inputVec,buttons

def slowDownSim(rate=0.00025):
  set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
  get_physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
  time_step = Float64(rate) #max_time_step
  max_update_rate = Float64(1000.0)
  gravity = Vector3()
  gravity.x = 0.0
  gravity.y = 0.0
  gravity.z = -9.8
  ode_config = get_physics().ode_config
  set_physics(time_step.data, max_update_rate.data, gravity, ode_config)

def main(argv):
  """Runs the locomotion controller example."""
  del argv # unused

  # Construct simulator
  if FLAGS.show_gui and not FLAGS.use_real_robot:
    p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
  else:
    p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
  p.setPhysicsEngineParameter(numSolverIterations=30)
  p.setTimeStep(0.001)
  p.setGravity(0, 0, -9.8)
  p.setPhysicsEngineParameter(enableConeFriction=0)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf")
  
  UPDATE_RATE = 100
  
  if FLAGS.use_gazebo:
    from motion_imitation.robots.A1RobotGazebo import A1RobotGazebo
    if FLAGS.pos_control:
      robot = A1RobotGazebo(
          pybullet_client=p,
          motor_control_mode=robot_config.MotorControlMode.HYBRID,
          enable_action_interpolation=False,
          time_step=0.002,
          action_repeat=1,
          position_control=True,
          update_rate=UPDATE_RATE)
    else:
      robot = A1RobotGazebo(
          pybullet_client=p,
          motor_control_mode=robot_config.MotorControlMode.HYBRID,
          enable_action_interpolation=False,
          time_step=0.002,
          action_repeat=1)
  elif not FLAGS.use_real_robot: #pybullet
    if FLAGS.pos_control:
      raise Exception("Position control not implemented for pybullet")
    else:
          robot = a1.A1(p,
                  motor_control_mode=robot_config.MotorControlMode.HYBRID,
                  enable_action_interpolation=False,
                  reset_time=2,
                  time_step=0.002,
                  action_repeat=1)
  else:
    from motion_imitation.robots.a1_robot import A1Robot
    if FLAGS.pos_control:
      robot = A1Robot(
          pybullet_client=p,
          motor_control_mode=robot_config.MotorControlMode.HYBRID,
          enable_action_interpolation=False,
          time_step=0.002,
          action_repeat=1,
          position_control=True,
          update_rate=UPDATE_RATE)
    else:
      robot = A1Robot(
          pybullet_client=p,
          motor_control_mode=robot_config.MotorControlMode.HYBRID,
          enable_action_interpolation=False,
          time_step=0.002,
          action_repeat=1)


  if FLAGS.use_gamepad:
      gamepad = gamepad_reader.Gamepad()
      command_function = gamepad.get_command
  else:
      command_function = _generate_example_linear_angular_speed

  timeline, com_vels, imu_rates = [], [], []

  if not FLAGS.pos_control:
    controller = _setup_controller(robot)

    controller.reset()

    robot.controller = controller
    if FLAGS.logdir:
      logdir = os.path.join(FLAGS.logdir,
                            datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
      os.makedirs(logdir)


    start_time = robot.GetTimeSinceReset()
    current_time = start_time

  else:
    start_time = robot.GetTimeSinceReset()
    current_time = start_time
    inputCommand = InputCommand([0,0],0)
    inputCommand.useIMU = True
    robot.sendControllerCommand(inputCommand)

    current_time = start_time
    #slowDownSim(0.0001)

  if FLAGS.use_real_robot or FLAGS.use_gazebo:
    r = rospy.Rate(UPDATE_RATE)

  while not rospy.is_shutdown() and current_time - start_time < FLAGS.max_time_secs:
      start_time_robot = current_time
      start_time_wall = time.time()
      # Updates the controller behavior parameters.
      lin_speed, ang_speed, e_stop = command_function(current_time)

      if e_stop:
        logging.info("E-stop kicked, exiting...")
        break
      
      inputCommand = InputCommand(lin_speed,ang_speed)
      if FLAGS.pos_control:
        inputCommand.useIMU = True
      if not robot.sendControllerCommand(inputCommand):
        break

      timeline.append(current_time)
      com_vels.append(np.array(robot.GetBaseVelocity()).copy())
      imu_rates.append(np.array(robot.GetBaseRollPitchYawRate()).copy())

      current_time = robot.GetTimeSinceReset()
      rollPitch = robot.GetBaseRollPitchYaw()

      # Stop simulation if rollover
      if not FLAGS.pos_control and (abs(rollPitch[0])> 0.5 or abs(rollPitch[1])> 0.5):
        print("Rolled over")
        break

      if not FLAGS.use_real_robot:
        expected_duration = current_time - start_time_robot
        actual_duration = time.time() - start_time_wall
        if actual_duration < expected_duration:
          time.sleep(expected_duration - actual_duration)
      if FLAGS.use_real_robot or FLAGS.use_gazebo:
        r.sleep()



  if FLAGS.use_gamepad:
    gamepad.stop()
  
  # Get robot to starting position after the end
  if FLAGS.use_real_robot:
    # start servopy
    print("running servo")
    node_process = Popen(shlex.split('rosrun unitree_controller servopy.py'))

  ############# PLOTTING ##############################
  if not FLAGS.pos_control:
    temp=np.zeros((len(robot.actions),12))
    for i in range(len(robot.actions)):
      temp[i][:] = robot.actions[i][4:60:5]

    plt.figure()
    plt.plot(timeline, temp)
    plt.legend(['FR_hip_torque','FR_upper_torque','FR_lower_torque',
                'FL_hip_torque','FL_upper_torque','FL_lower_torque',
                'RR_hip_torque','RR_upper_torque','RR_lower_torque',
                'RL_hip_torque','RL_upper_torque','RL_lower_torque'])

    plt.figure()
    plt.plot(timeline, com_vels)
    plt.legend(['Vx','Vy','Vz'])

    plt.figure()
    plt.plot(timeline, imu_rates)
    plt.legend(['Wx','Wy','Wz'])

    plt.figure()
    plt.plot(timeline, controller._stance_leg_controller._desired_ddq_array)
    plt.legend(['dd_x','dd_y','dd_z','dd_roll','dd_pitch','dd_yaw'])
    plt.show()

  #######################################################

  if FLAGS.logdir:
    np.savez(os.path.join(logdir, 'action.npz'),
             action=robot.actions,
             com_vels=com_vels,
             imu_rates=imu_rates)
    logging.info("logged to: {}".format(logdir))


if __name__ == "__main__":
  app.run(main)
