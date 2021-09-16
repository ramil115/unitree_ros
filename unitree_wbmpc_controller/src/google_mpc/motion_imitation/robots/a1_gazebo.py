import re
import rospy
from unitree_legged_msgs.msg import LowState
from unitree_legged_msgs.msg import LowCmd
from unitree_legged_msgs.msg import MotorState
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import Imu,Joy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import time
import numpy as np

class a1_ros:
    def __init__(self, rname, position_control=False, update_rate=1000):

        self.np = rospy.init_node('a1_ros', anonymous=True)
        self.robot_name = rname
        self.pos_control = position_control

        self.kps = np.array([100]*12)
        self.kds = np.array([1., 2., 2., 1., 2., 2., 1., 2., 2., 1., 2., 2.])

        controller_names =  ["_gazebo/FR_hip_controller",
                "_gazebo/FR_thigh_controller",
                "_gazebo/FR_calf_controller",
                "_gazebo/FL_hip_controller",
                "_gazebo/FL_thigh_controller",
                "_gazebo/FL_calf_controller",
                "_gazebo/RR_hip_controller",
                "_gazebo/RR_thigh_controller",
                "_gazebo/RR_calf_controller",
                "_gazebo/RL_hip_controller",
                "_gazebo/RL_thigh_controller",
                "_gazebo/RL_calf_controller"
                ]

        callbacks = [self.FRhipCallback,
                    self.FRthighCallback,
                    self.FRcalfCallback,
                    self.FLhipCallback,
                    self.FLthighCallback,
                    self.FLcalfCallback,
                    self.RRhipCallback,
                    self.RRthighCallback,
                    self.RRcalfCallback,
                    self.RLhipCallback,
                    self.RLthighCallback,
                    self.RLcalfCallback]

        self.footForceThreshold = 0.01
        self.lowState = LowState()
        self.lowCmd = LowCmd()
        self.paramInit()
        self.imu_sub = rospy.Subscriber("/trunk_imu", Imu, self.imuCallback)
        self.footForce_sub = [None] * 4

        self.footForce_sub[0] = rospy.Subscriber("/visual/FR_foot_contact/the_force", WrenchStamped,
                                                self.FRfootCallback)
        self.footForce_sub[1] = rospy.Subscriber("/visual/FL_foot_contact/the_force", WrenchStamped,
                                                self.FLfootCallback)
        self.footForce_sub[2] = rospy.Subscriber("/visual/RR_foot_contact/the_force", WrenchStamped,
                                                self.RRfootCallback)
        self.footForce_sub[3] = rospy.Subscriber("/visual/RL_foot_contact/the_force", WrenchStamped,
                                                self.RLfootCallback)


        self.servo_sub = [rospy.Subscriber("/" + self.robot_name + controller_name +"/state",
                                            MotorState, callback) for controller_name,callback in zip(controller_names,callbacks)]
        if position_control:
            from .PositionalControl.RobotController import RobotController
            from .PositionalControl.InverseKinematics import robot_IK
            body = [0.366, 0.094]   
            legs = [0.,0.08505, 0.2, 0.2] 

            self.a1_robot = RobotController.Robot(body, legs, True, 1/update_rate)
            self.inverseKinematics = robot_IK.InverseKinematics(body, legs)

            self.orientation_sub = rospy.Subscriber("/trunk_imu", Imu, self.a1_robot.imu_orientation)
        self.servo_pub = [rospy.Publisher("/" + self.robot_name + controller_name +"/command", MotorCmd, queue_size=10) for controller_name in controller_names]
        self.actions = []
        time.sleep(2)
        self.resetTime = rospy.get_time()

    def imuCallback(self, msg):
        # print("IMU Callback run")
        self.lowState.imu.quaternion[0] = msg.orientation.w
        self.lowState.imu.quaternion[1] = msg.orientation.x
        self.lowState.imu.quaternion[2] = msg.orientation.y
        self.lowState.imu.quaternion[3] = msg.orientation.z

        self.lowState.imu.gyroscope[0] = msg.angular_velocity.x
        self.lowState.imu.gyroscope[1] = msg.angular_velocity.y
        self.lowState.imu.gyroscope[2] = msg.angular_velocity.z

        self.lowState.imu.accelerometer[0] = msg.linear_acceleration.x
        self.lowState.imu.accelerometer[1] = msg.linear_acceleration.y
        self.lowState.imu.accelerometer[2] = msg.linear_acceleration.z
    
    def FRhipCallback(self, msg):
        self.lowState.motorState[0].mode = msg.mode
        self.lowState.motorState[0].q = msg.q
        self.lowState.motorState[0].dq = msg.dq
        self.lowState.motorState[0].tauEst = msg.tauEst

    def FRthighCallback(self, msg):
        self.lowState.motorState[1].mode = msg.mode
        self.lowState.motorState[1].q = msg.q
        self.lowState.motorState[1].dq = msg.dq
        self.lowState.motorState[1].tauEst = msg.tauEst

    def FRcalfCallback(self, msg):
        self.lowState.motorState[2].mode = msg.mode
        self.lowState.motorState[2].q = msg.q
        self.lowState.motorState[2].dq = msg.dq
        self.lowState.motorState[2].tauEst = msg.tauEst

    def FLhipCallback(self, msg):
        self.lowState.motorState[3].mode = msg.mode
        self.lowState.motorState[3].q = msg.q
        self.lowState.motorState[3].dq = msg.dq
        self.lowState.motorState[3].tauEst = msg.tauEst

    def FLthighCallback(self, msg):
        self.lowState.motorState[4].mode = msg.mode
        self.lowState.motorState[4].q = msg.q
        self.lowState.motorState[4].dq = msg.dq
        self.lowState.motorState[4].tauEst = msg.tauEst

    def FLcalfCallback(self, msg):
        self.lowState.motorState[5].mode = msg.mode
        self.lowState.motorState[5].q = msg.q
        self.lowState.motorState[5].dq = msg.dq
        self.lowState.motorState[5].tauEst = msg.tauEst

    def RRhipCallback(self, msg):
        self.lowState.motorState[6].mode = msg.mode
        self.lowState.motorState[6].q = msg.q
        self.lowState.motorState[6].dq = msg.dq
        self.lowState.motorState[6].tauEst = msg.tauEst

    def RRthighCallback(self, msg):
        self.lowState.motorState[7].mode = msg.mode
        self.lowState.motorState[7].q = msg.q
        self.lowState.motorState[7].dq = msg.dq
        self.lowState.motorState[7].tauEst = msg.tauEst

    def RRcalfCallback(self, msg):
        self.lowState.motorState[8].mode = msg.mode
        self.lowState.motorState[8].q = msg.q
        self.lowState.motorState[8].dq = msg.dq
        self.lowState.motorState[8].tauEst = msg.tauEst

    def RLhipCallback(self, msg):
        self.lowState.motorState[9].mode = msg.mode
        self.lowState.motorState[9].q = msg.q
        self.lowState.motorState[9].dq = msg.dq
        self.lowState.motorState[9].tauEst = msg.tauEst

    def RLthighCallback(self, msg):
        self.lowState.motorState[10].mode = msg.mode
        self.lowState.motorState[10].q = msg.q
        self.lowState.motorState[10].dq = msg.dq
        self.lowState.motorState[10].tauEst = msg.tauEst

    def RLcalfCallback(self, msg):
        self.lowState.motorState[11].mode = msg.mode
        self.lowState.motorState[11].q = msg.q
        self.lowState.motorState[11].dq = msg.dq
        self.lowState.motorState[11].tauEst = msg.tauEst

    def FRfootCallback(self, msg):
        self.lowState.eeForce[0].x = msg.wrench.force.x
        self.lowState.eeForce[0].y = msg.wrench.force.y
        self.lowState.eeForce[0].z = msg.wrench.force.z
        self.lowState.footForce[0] = msg.wrench.force.z

    def FLfootCallback(self, msg):
        self.lowState.eeForce[1].x = msg.wrench.force.x
        self.lowState.eeForce[1].y = msg.wrench.force.y
        self.lowState.eeForce[1].z = msg.wrench.force.z
        self.lowState.footForce[1] = msg.wrench.force.z

    def RRfootCallback(self, msg):
        self.lowState.eeForce[2].x = msg.wrench.force.x
        self.lowState.eeForce[2].y = msg.wrench.force.y
        self.lowState.eeForce[2].z = msg.wrench.force.z
        self.lowState.footForce[2] = msg.wrench.force.z

    def RLfootCallback(self, msg):
        self.lowState.eeForce[3].x = msg.wrench.force.x
        self.lowState.eeForce[3].y = msg.wrench.force.y
        self.lowState.eeForce[3].z = msg.wrench.force.z
        self.lowState.footForce[3] = msg.wrench.force.z

    def paramInit(self):
        for i in range(4):
            self.lowCmd.motorCmd[i*3+0].mode = 0x0A
            # self.lowCmd.motorCmd[i*3+0].Kp = 70
            # self.lowCmd.motorCmd[i*3+0].dq = 0
            # self.lowCmd.motorCmd[i*3+0].Kd = 3
            # self.lowCmd.motorCmd[i*3+0].tau = 0
            self.lowCmd.motorCmd[i*3+1].mode = 0x0A
            # self.lowCmd.motorCmd[i*3+1].Kp = 180
            # self.lowCmd.motorCmd[i*3+1].dq = 0
            # self.lowCmd.motorCmd[i*3+1].Kd = 8
            # self.lowCmd.motorCmd[i*3+1].tau = 0
            self.lowCmd.motorCmd[i*3+2].mode = 0x0A
            # self.lowCmd.motorCmd[i*3+2].Kp = 300
            # self.lowCmd.motorCmd[i*3+2].dq = 0
            # self.lowCmd.motorCmd[i*3+2].Kd = 15
            # self.lowCmd.motorCmd[i*3+2].tau = 0
    
    def getJointStates(self): 
        return [ [self.lowState.motorState[i].q,self.lowState.motorState[i].dq] for i in range(12)]
    
    def getBaseAngularVelocity(self):
        return self.lowState.imu.gyroscope

    def getContactPoints(self):
        contacts = [False]*4
        for i in range(4):
            if self.lowState.footForce[i]>self.footForceThreshold:
                contacts[i] = True

        return contacts

    def getBaseOrientation(self):
        return self.lowState.imu.quaternion

    def receive_observation(self):
        return self.lowState

    def send_command(self, command):
        if self.pos_control:
            self.sendJointCmd(command)
        else:
            self.sendTorqueCmd(command)
    
    def sendJointCmd(self,cmd):
        for motor_id in range(12):
            self.lowCmd.motorCmd[motor_id].q=cmd[motor_id]
            self.lowCmd.motorCmd[motor_id].Kp=self.kps[motor_id]
            self.lowCmd.motorCmd[motor_id].Kd=self.kds[motor_id]
            self.lowCmd.motorCmd[motor_id].tau=0
        for m in range(12):
            self.servo_pub[m].publish(self.lowCmd.motorCmd[m])


    def sendTorqueCmd(self,cmd):
        for motor_id in range(12):
            self.lowCmd.motorCmd[motor_id].q=cmd[motor_id * 5]
            self.lowCmd.motorCmd[motor_id].Kp=cmd[motor_id * 5+1]
            self.lowCmd.motorCmd[motor_id].Kd=cmd[motor_id * 5+3]
            self.lowCmd.motorCmd[motor_id].tau=cmd[motor_id * 5+4]
        for m in range(12):
            self.servo_pub[m].publish(self.lowCmd.motorCmd[m])

    def setMovement(self,type,inputVec=[0,0,1,0,0,1,0,0],buttons=[False]*8):
        self.a1_robot.set_movement(type,inputVec,buttons)
    
    def stepPosControl(self):
        self.leg_positions = self.a1_robot.run()
        self.a1_robot.change_controller()

    def getPositionCommand(self):
        dx = self.a1_robot.state.body_local_position[0]
        dy = self.a1_robot.state.body_local_position[1]
        dz = self.a1_robot.state.body_local_position[2]
        
        roll = self.a1_robot.state.body_local_orientation[0]
        pitch = self.a1_robot.state.body_local_orientation[1]
        yaw = self.a1_robot.state.body_local_orientation[2]

        try:
            joint_angles = self.inverseKinematics.inverse_kinematics(self.leg_positions,
                                dx, dy, dz, roll, pitch, yaw)
            return joint_angles
        except:
            print("POSITION FAIL")
            return None

    def getTimeSinceReset(self):
        return rospy.get_time() - self.resetTime

    def sendControllerCommand(self,inputCommand):
        inputVec,buttons = inputCommand.transformToPosControl()
        self.setMovement("trot",inputVec,buttons)
        self.stepPosControl()
        command = self.getPositionCommand()

        if command != None:
            self.send_command(command)
            self.actions.append(command)
            return True
        else:
            return False

    