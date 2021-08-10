import rospy
from unitree_legged_msgs.msg import LowState
from unitree_legged_msgs.msg import LowCmd
from unitree_legged_msgs.msg import MotorState
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped



class a1_ros:
    def __init__(self, rname):
        self.np = rospy.init_node('a1_ros', anonymous=True)
        self.robot_name = rname
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
        self.servo_sub = [None] * 12
        self.servo_sub[0] = rospy.Subscriber("/" + self.robot_name + "_gazebo/FR_hip_controller/state",
                                             MotorState, self.FRhipCallback)
        self.servo_sub[1] = rospy.Subscriber("/" + self.robot_name + "_gazebo/FR_thigh_controller/state",
                                             MotorState, self.FRthighCallback)
        self.servo_sub[2] = rospy.Subscriber("/" + self.robot_name + "_gazebo/FR_calf_controller/state",
                                             MotorState, self.FRcalfCallback)

        self.servo_sub[3] = rospy.Subscriber("/" + self.robot_name + "_gazebo/FL_hip_controller/state",
                                             MotorState, self.FLhipCallback)
        self.servo_sub[4] = rospy.Subscriber("/" + self.robot_name + "_gazebo/FL_thigh_controller/state",
                                             MotorState, self.FLthighCallback)
        self.servo_sub[5] = rospy.Subscriber("/" + self.robot_name + "_gazebo/FL_calf_controller/state",
                                             MotorState, self.FLcalfCallback)

        self.servo_sub[6] = rospy.Subscriber("/" + self.robot_name + "_gazebo/RR_hip_controller/state",
                                             MotorState, self.RRhipCallback)
        self.servo_sub[7] = rospy.Subscriber("/" + self.robot_name + "_gazebo/RR_thigh_controller/state",
                                             MotorState, self.RRthighCallback)
        self.servo_sub[8] = rospy.Subscriber("/" + self.robot_name + "_gazebo/RR_calf_controller/state",
                                             MotorState, self.RRcalfCallback)

        self.servo_sub[9] = rospy.Subscriber("/" + self.robot_name + "_gazebo/RL_hip_controller/state",
                                             MotorState, self.RLhipCallback)
        self.servo_sub[10] = rospy.Subscriber("/" + self.robot_name + "_gazebo/RL_thigh_controller/state",
                                              MotorState, self.RLthighCallback)
        self.servo_sub[11] = rospy.Subscriber("/" + self.robot_name + "_gazebo/RL_calf_controller/state",
                                              MotorState, self.RLcalfCallback)
        
        self.servo_pub = [None] * 12

        self.servo_pub[0] = rospy.Publisher("/" + self.robot_name + "_gazebo/FR_hip_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[1] = rospy.Publisher("/" + self.robot_name + "_gazebo/FR_thigh_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[2] = rospy.Publisher("/" + self.robot_name + "_gazebo/FR_calf_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[3] = rospy.Publisher("/" + self.robot_name + "_gazebo/FL_hip_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[4] = rospy.Publisher("/" + self.robot_name + "_gazebo/FL_thigh_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[5] = rospy.Publisher("/" + self.robot_name + "_gazebo/FL_calf_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[6] = rospy.Publisher("/" + self.robot_name + "_gazebo/RR_hip_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[7] = rospy.Publisher("/" + self.robot_name + "_gazebo/RR_thigh_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[8] = rospy.Publisher("/" + self.robot_name + "_gazebo/RR_calf_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[9] = rospy.Publisher("/" + self.robot_name + "_gazebo/RL_hip_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[10] = rospy.Publisher("/" + self.robot_name + "_gazebo/RL_thigh_controller/command", MotorCmd, queue_size=10)
        self.servo_pub[11] = rospy.Publisher("/" + self.robot_name + "_gazebo/RL_calf_controller/command", MotorCmd, queue_size=10)



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
    
    def sendTorqueCmd(self,torques):
        for j in range(12):
            self.lowCmd.motorCmd[j].tau = torques[j]
        for m in range(12):
            self.servo_pub[m].publish(self.lowCmd.motorCmd[m])
