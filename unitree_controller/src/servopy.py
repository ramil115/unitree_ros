#!/usr/bin/env python3

import rospy
from unitree_legged_msgs.msg import LowCmd
from unitree_legged_msgs.msg import LowState
from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import MotorState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped


servo_pub = [None] * 12
lowCmd = LowCmd()
lowState = LowState()


class multiThread:
    def __init__(self, rname):
        self.robot_name = rname
        global lowState
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

    def imuCallback(self,msg):
        lowState.imu.quaternion[0] = msg.orientation.w
        lowState.imu.quaternion[1] = msg.orientation.x
        lowState.imu.quaternion[2] = msg.orientation.y
        lowState.imu.quaternion[3] = msg.orientation.z

        lowState.imu.gyroscope[0] = msg.angular_velocity.x
        lowState.imu.gyroscope[1] = msg.angular_velocity.y
        lowState.imu.gyroscope[2] = msg.angular_velocity.z

        lowState.imu.accelerometer[0] = msg.linear_acceleration.x
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z

    def FRhipCallback(self,msg):
        lowState.motorState[0].mode = msg.mode
        lowState.motorState[0].q = msg.q
        lowState.motorState[0].dq = msg.dq
        lowState.motorState[0].tauEst = msg.tauEst

    def FRthighCallback(self,msg):
        lowState.motorState[1].mode = msg.mode
        lowState.motorState[1].q = msg.q
        lowState.motorState[1].dq = msg.dq
        lowState.motorState[1].tauEst = msg.tauEst

    def FRcalfCallback(self,msg):
        lowState.motorState[2].mode = msg.mode
        lowState.motorState[2].q = msg.q
        lowState.motorState[2].dq = msg.dq
        lowState.motorState[2].tauEst = msg.tauEst

    def FLhipCallback(self,msg):
        lowState.motorState[3].mode = msg.mode
        lowState.motorState[3].q = msg.q
        lowState.motorState[3].dq = msg.dq
        lowState.motorState[3].tauEst = msg.tauEst

    def FLthighCallback(self,msg):
        lowState.motorState[4].mode = msg.mode
        lowState.motorState[4].q = msg.q
        lowState.motorState[4].dq = msg.dq
        lowState.motorState[4].tauEst = msg.tauEst

    def FLcalfCallback(self,msg):
        lowState.motorState[5].mode = msg.mode
        lowState.motorState[5].q = msg.q
        lowState.motorState[5].dq = msg.dq
        lowState.motorState[5].tauEst = msg.tauEst

    def RRhipCallback(self,msg):
        lowState.motorState[6].mode = msg.mode
        lowState.motorState[6].q = msg.q
        lowState.motorState[6].dq = msg.dq
        lowState.motorState[6].tauEst = msg.tauEst

    def RRthighCallback(self,msg):
        lowState.motorState[7].mode = msg.mode
        lowState.motorState[7].q = msg.q
        lowState.motorState[7].dq = msg.dq
        lowState.motorState[7].tauEst = msg.tauEst

    def RRcalfCallback(self,msg):
        lowState.motorState[8].mode = msg.mode
        lowState.motorState[8].q = msg.q
        lowState.motorState[8].dq = msg.dq
        lowState.motorState[8].tauEst = msg.tauEst

    def RLhipCallback(self,msg):
        lowState.motorState[9].mode = msg.mode
        lowState.motorState[9].q = msg.q
        lowState.motorState[9].dq = msg.dq
        lowState.motorState[9].tauEst = msg.tauEst

    def RLthighCallback(self,msg):
        lowState.motorState[10].mode = msg.mode
        lowState.motorState[10].q = msg.q
        lowState.motorState[10].dq = msg.dq
        lowState.motorState[10].tauEst = msg.tauEst

    def RLcalfCallback(self,msg):
        lowState.motorState[11].mode = msg.mode
        lowState.motorState[11].q = msg.q
        lowState.motorState[11].dq = msg.dq
        lowState.motorState[11].tauEst = msg.tauEst

    def FRfootCallback(self,msg):
        lowState.eeForce[0].x = msg.wrench.force.x
        lowState.eeForce[0].y = msg.wrench.force.y
        lowState.eeForce[0].z = msg.wrench.force.z
        lowState.footForce[0] = msg.wrench.force.z

    def FLfootCallback(self,msg):
        lowState.eeForce[1].x = msg.wrench.force.x
        lowState.eeForce[1].y = msg.wrench.force.y
        lowState.eeForce[1].z = msg.wrench.force.z
        lowState.footForce[1] = msg.wrench.force.z

    def RRfootCallback(self,msg):
        lowState.eeForce[2].x = msg.wrench.force.x
        lowState.eeForce[2].y = msg.wrench.force.y
        lowState.eeForce[2].z = msg.wrench.force.z
        lowState.footForce[2] = msg.wrench.force.z

    def RLfootCallback(self,msg):
        lowState.eeForce[3].x = msg.wrench.force.x
        lowState.eeForce[3].y = msg.wrench.force.y
        lowState.eeForce[3].z = msg.wrench.force.z
        lowState.footForce[3] = msg.wrench.force.z

def paramInit():
    global lowCmd, lowState
    for i in range(4):
        lowCmd.motorCmd[i*3+0].mode = 0x0A
        lowCmd.motorCmd[i*3+0].Kp = 70
        lowCmd.motorCmd[i*3+0].dq = 0
        lowCmd.motorCmd[i*3+0].Kd = 3
        lowCmd.motorCmd[i*3+0].tau = 0
        lowCmd.motorCmd[i*3+1].mode = 0x0A
        lowCmd.motorCmd[i*3+1].Kp = 180
        lowCmd.motorCmd[i*3+1].dq = 0
        lowCmd.motorCmd[i*3+1].Kd = 8
        lowCmd.motorCmd[i*3+1].tau = 0
        lowCmd.motorCmd[i*3+2].mode = 0x0A
        lowCmd.motorCmd[i*3+2].Kp = 300
        lowCmd.motorCmd[i*3+2].dq = 0
        lowCmd.motorCmd[i*3+2].Kd = 15
        lowCmd.motorCmd[i*3+2].tau = 0
    for i in range(12):
        lowCmd.motorCmd[i].q = lowState.motorState[i].q

def stand():   
    pos = [0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 
                      0.0, 0.9, -1.8, 0.0, 0.9, -1.8]
    moveAllPosition(pos, 2*100);


def motion_init():
    paramInit()
    stand()


def sendServoCmd():
    for m in range(12):
        servo_pub[m].publish(lowCmd.motorCmd[m])
    rospy.sleep(0.01)


def moveAllPosition(targetPos, duration):
    global lowCmd
    lastPos = [None] *12
    for j in range(12):
        lastPos[j] = lowState.motorState[j].q
    for i in range(duration):
        percent = i/duration
        for j in range(12):
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent 
        sendServoCmd()



def servo():

    rospy.init_node('listener', anonymous=True)
    mt = multiThread("a1")
    rospy.sleep(0.2)
    # print(lowState.motorState[0])

    robot_name = "a1"
    # for rviz visualization
    lowState_pub = rospy.Publisher("/" + robot_name + "_gazebo/lowState/state", LowState, queue_size=10) 

    global servo_pub
    servo_pub[0] = rospy.Publisher("/" + robot_name + "_gazebo/FR_hip_controller/command", MotorCmd, queue_size=10)
    servo_pub[1] = rospy.Publisher("/" + robot_name + "_gazebo/FR_thigh_controller/command", MotorCmd, queue_size=10)
    servo_pub[2] = rospy.Publisher("/" + robot_name + "_gazebo/FR_calf_controller/command", MotorCmd, queue_size=10)
    servo_pub[3] = rospy.Publisher("/" + robot_name + "_gazebo/FL_hip_controller/command", MotorCmd, queue_size=10)
    servo_pub[4] = rospy.Publisher("/" + robot_name + "_gazebo/FL_thigh_controller/command", MotorCmd, queue_size=10)
    servo_pub[5] = rospy.Publisher("/" + robot_name + "_gazebo/FL_calf_controller/command", MotorCmd, queue_size=10)
    servo_pub[6] = rospy.Publisher("/" + robot_name + "_gazebo/RR_hip_controller/command", MotorCmd, queue_size=10)
    servo_pub[7] = rospy.Publisher("/" + robot_name + "_gazebo/RR_thigh_controller/command", MotorCmd, queue_size=10)
    servo_pub[8] = rospy.Publisher("/" + robot_name + "_gazebo/RR_calf_controller/command", MotorCmd, queue_size=10)
    servo_pub[9] = rospy.Publisher("/" + robot_name + "_gazebo/RL_hip_controller/command", MotorCmd, queue_size=10)
    servo_pub[10] = rospy.Publisher("/" + robot_name + "_gazebo/RL_thigh_controller/command", MotorCmd, queue_size=10)
    servo_pub[11] = rospy.Publisher("/" + robot_name + "_gazebo/RL_calf_controller/command", MotorCmd, queue_size=10)

    motion_init()

    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     lowState_pub.publish(lowState)
        # sendServoCmd()
        # rate.sleep()



if __name__ == '__main__':    
    try:
        servo()
    except rospy.ROSInterruptException:
        pass