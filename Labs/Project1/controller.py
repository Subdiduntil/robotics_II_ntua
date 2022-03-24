#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
import matplotlib.pyplot as plt

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)

        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):

        # set configuration
        self.joint_angpos = [0, 0.75, 0, 1.5, 0, 0.75, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        #################### Path Planning of Robot Manipulator (Task Space)  #########################
        ### Cubic Interpolation for Continuous Joint Positions qi and Joint Velocities ###
        def path_planning(pA, T, start = 0):

            # Time Intervals of Simulation
            Tmax = int(600*T + 1)
            t = [(i/600) for i in range(int(Tmax))] # time vector


            distance_AB = 0.4

            # Posiiton of End-Effector after Initialisation
            pE = pA # coordinates of start point pA


            # Coordinates of desired Start Point (pA)
            xd_0 = pE.item(0)
            yd_0 = pE.item(1)
            zd_0 = pE.item(2)


            # Coordinates of desired Final Point (pB) - Only y-coordinate changes
            if start == 1:
                yd_f = pE[1]+distance_AB/2 # (pA+pB)/2 --> pA
            else:
                yd_f = pE[1]-distance_AB # pA --> pB

            # Cubic Interpolation - Polynomial Coefficients (Task Space)
            a0 = yd_0
            a2 = 3/(T**2)*(yd_0-yd_f) 
            a3 = -2/(T**3)*(yd_0-yd_f)

            # Desired Position of End-Effector - on line AB
            xd = xd_0*np.ones(Tmax)
            yd = a0 + a2*np.power(t, 2) + a3*np.power(t, 3) # interpolation
            zd = zd_0*np.ones(Tmax)

            # Desired Velocity of End-Effector
            # Due to continuity in velocity --> V0 = Vf = 0
            Vxd = np.zeros(Tmax)
            Vyd = (2*a2)*t + (3*a3)*np.power(t, 2) #differentiate
            Vzd = np.zeros(Tmax)

            return xd, yd, zd, Vxd, Vyd, Vzd

        
        def redundant_algorithm(tk, xd, yd, zd, Vxd, Vyd, Vzd, data, collect = False):
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            self.A01 = self.kinematics.tf_A01(self.joint_angpos)
            self.A02 = self.kinematics.tf_A02(self.joint_angpos)
            self.A03 = self.kinematics.tf_A03(self.joint_angpos)
            self.A04 = self.kinematics.tf_A04(self.joint_angpos)
            self.A05 = self.kinematics.tf_A05(self.joint_angpos)
            self.A06 = self.kinematics.tf_A06(self.joint_angpos)
            self.A07 = self.kinematics.tf_A07(self.joint_angpos)

            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)
            # pseudoinverse jacobian
            pinvJ = pinv(J)

            ##### 1st TASK: PATH PLANNING #####

            # desired end-effector velocity
            v1d = np.matrix([[Vxd[tk]],[Vyd.item(tk)],[Vzd[tk]]])
            # desired end-effector position
            p1d = np.matrix([[xd[tk]],\
                                [yd.item(tk)],\
                                [zd[tk]]])

            # real position of end-effector
            f1 = self.A07[:-1, 3]

            K1 = 130 # control gain

            qdot1 = pinvJ @ (v1d + K1*(p1d - f1))

            ##### 2nd TASK: OBSTACLE AVOIDANCE #####

            # Get obstacles position
            green_obst = self.model_states.pose[1].position.y
            red_obst = self.model_states.pose[2].position.y

            middle_obst = (green_obst + red_obst)/2 # middle of the two obstacles

            
            # Criteria Functions to Optimize (Minimize): Distance of Joints from Middle of Obstacles
            Kc = 100
            V1 = (1/2) * Kc * ((self.A03[1,3] - middle_obst) ** 2) #distance from q3
            V2 = (1/2) * Kc * ((self.A04[1,3] - middle_obst) ** 2) #distance from q4

            # Gradient of V1, V2 --> reference joint velocities of 2nd Task

            # Fetch some useful data...
            l2 = self.kinematics.l2
            l3 = self.kinematics.l3
            q1 = self.joint_angpos[0]
            q2 = self.joint_angpos[1]
            q3 = self.joint_angpos[2]
            c1 = np.cos(q1)
            c2 = np.cos(q2)
            c3 = np.cos(q3)
            s1 = np.sin(q1)
            s2 = np.sin(q2)
            s3 = np.sin(q3)

            # Distance from joint q3
            reference1 = np.zeros((7,1))
            reference1[0] = -Kc * (self.A03[1,3] - middle_obst) * l2*c1*s2
            reference1[1] = -Kc * (self.A03[1,3] - middle_obst) * l2*c2*s1 

            # Distance from joint q4
            reference2 = np.zeros((7,1))
            reference2[0] = -Kc * (self.A04[1,3] - middle_obst) * (l2*c1*s2 - l3*(s1*s3 - c1*c2*c3))
            reference2[1] = -Kc * (self.A04[1,3] - middle_obst) * (l2*c2*s1 - l3*c3*s1*s2)
            reference2[2] = -Kc * (self.A04[1,3] - middle_obst) * (l3*(c1*c3 - c2*s1*s3))

            K2 = [15, 15] # some gains
            total_reference = K2[0]*reference1 + K2[1]*reference2

            qdot2 = (np.eye(7) - np.dot(pinvJ, J)) @ total_reference

            # Finally, decide a threshold when to avoid obstacles
            threshold = 0.02

            if (max(V1, V2) >= threshold):
                for i in range(7):
                    self.joint_angvel[i] = qdot1[i,0] + qdot2[i,0] # do both tasks
            else:
                for i in range(7):
                    self.joint_angvel[i] = qdot1[i,0] + qdot2[i,0] # do only path planning

            ############################################################################

            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9
            # Integration
            self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )

            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            #self.pub_rate.sleep()

            # Collect data to plot
            """if collect == 1:
                data[0].append(self.joint_angpos[0])
                data[1].append(self.joint_angpos[1])
                data[2].append(self.joint_angpos[2])
                data[3].append(self.joint_angpos[3])
                data[4].append(self.joint_angpos[4])
                data[5].append(self.joint_angpos[5])
                data[6].append(self.joint_angpos[6])
                data[7].append(f1[0])
                data[8].append(f1[1])
                data[9].append(f1[2])
                data[10].append(v1d[0])
                data[11].append(v1d[1])
                data[12].append(v1d[2])
                data[13].append(p1d[1]-f1[1])
                data[14].append(V1)
                data[15].append(V2)
                data[16].append(self.A03[1,3] - green_obst)
                data[17].append(self.A03[1,3] - red_obst)
                data[18].append(self.A04[1,3] - green_obst)
                data[19].append(self.A04[1,3] - red_obst)
                data[20].append(self.joint_angvel[0])
                data[21].append(self.joint_angvel[1])
                data[22].append(self.joint_angvel[2])
                data[23].append(self.joint_angvel[3])
                data[24].append(self.joint_angvel[4])
                data[25].append(self.joint_angvel[5])
                data[26].append(self.joint_angvel[6])
                return data"""
            return []

            

        T = 2.5 # duration of movement (pA+pB)/2 -> pA

        # Posiiton of End-Effector after Initialisation
        A07 = self.kinematics.tf_A07(self.joint_angpos)
        pE = A07[0:3,3] # coordinates of (pA+pB)/2

        # Firstly, we must path plan the movement from the middle (pA+pB)/2 to point A
        xd, yd, zd, Vxd, Vyd, Vzd = path_planning(pE, T, start = 1)

        tk = 0 #time counter to reverse movement between points A and B
        Tmax = int(600*T + 1)
        while (0<=tk<Tmax):
            collected = redundant_algorithm(tk, xd, yd, zd, Vxd, Vyd, Vzd, False)
            tk += 1


        # After that, the main algorithm runs endlessly
        T = 2.5 # duration of movement pA -> pB

        # Posiiton of End-Effector after Initialisation
        A07 = self.kinematics.tf_A07(self.joint_angpos)
        pE = A07[0:3,3] # coordinates of pA

        # Path Planning pA --> pB and vice versa
        xd, yd, zd, Vxd, Vyd, Vzd = path_planning(pE, T, start = 0)

        tk = 0 #time counter to reverse movement between points A and B
        A_to_B = True
        
        
        # Initalize data to plot
        """joint1 = []
        joint2 = []
        joint3 = []
        joint4 = [] 
        joint5 = []
        joint6 = []
        joint7 = []
        px = []
        py =  []
        pz = []
        Vx = []
        Vy = []
        Vz = []
        pos_error = []
        V1 = []
        V2 = []
        green_q3 = []
        red_q3 = []
        green_q4 = []
        red_q4 = []
        vjoint1 = []
        vjoint2 = []
        vjoint3 = []
        vjoint4 = [] 
        vjoint5 = []
        vjoint6 = []
        vjoint7 = []
        data = (joint1, joint2, joint3, joint4, joint5, joint6, joint7, px, py, pz,Vx, Vy, Vz, pos_error, V1, V2, green_q3, red_q3, green_q4, red_q4, vjoint1, vjoint2, vjoint3, vjoint4, vjoint5, vjoint6, vjoint7)
        """
        data = []
        # Redundant Kinematic Control
        while not rospy.is_shutdown():
            while (0<=tk<Tmax):
                    
                    collected = redundant_algorithm(tk, xd, yd, zd, Vxd, Vyd, Vzd, data, True)

                    if(A_to_B):
                        tk += 1
                    else:
                        tk -= 1

            # One entire movement betwwen A and B is finished, reverse direction of movement        
            # fix time counter
            if(A_to_B):
                tk -= 2
            else:
                tk += 2
            A_to_B = not A_to_B

        
        # Write data to plot
        """# Open files to write
        joint1_data = open('joint1.txt', 'w')
        joint2_data = open('joint2.txt', 'w')
        joint3_data = open('joint3.txt', 'w')
        joint4_data = open('joint4.txt', 'w')
        joint5_data = open('joint5.txt', 'w')
        joint6_data = open('joint6.txt', 'w')
        joint7_data = open('joint7.txt', 'w')
        vjoint1_data = open('vjoint1.txt', 'w')
        vjoint2_data = open('vjoint2.txt', 'w')
        vjoint3_data = open('vjoint3.txt', 'w')
        vjoint4_data = open('vjoint4.txt', 'w')
        vjoint5_data = open('vjoint5.txt', 'w')
        vjoint6_data = open('vjoint6.txt', 'w')
        vjoint7_data = open('vjoint7.txt', 'w')
        px_data = open('px.txt', 'w')
        py_data = open('py.txt', 'w')
        pz_data = open('pz.txt', 'w')
        vx_data = open('vx.txt', 'w')
        vy_data = open('vy.txt', 'w')
        vz_data = open('vz.txt', 'w')
        error_data = open('error.txt', 'w')
        V1_data = open('V1.txt','w')
        V2_data = open('V2.txt', 'w')
        greenq1_data = open('green_q1.txt', 'w')
        redq1_data = open('red_q1.txt', 'w')
        greenq2_data = open('green_q2.txt', 'w')
        redq2_data = open('red_q2.txt', 'w')

        for j in range(len(collected[0])):
            joint1_data.write(str(collected[0][j])+' ')
            joint2_data.write(str(collected[1][j])+' ')
            joint3_data.write(str(collected[2][j])+' ')
            joint4_data.write(str(collected[3][j])+' ')
            joint5_data.write(str(collected[4][j])+' ')
            joint6_data.write(str(collected[5][j])+' ')
            joint7_data.write(str(collected[6][j])+' ')
            px_data.write(str(collected[7][j])+' ')
            py_data.write(str(collected[8][j])+' ')
            pz_data.write(str(collected[9][j])+' ')
            vx_data.write(str(collected[10][j])+' ')
            vy_data.write(str(collected[11][j])+' ')
            vz_data.write(str(collected[12][j])+' ')
            error_data.write(str(collected[13][j])+' ')
            V1_data.write(str(collected[14][j])+' ')
            V2_data.write(str(collected[15][j])+' ')
            greenq1_data.write(str(collected[16][j])+' ')
            redq1_data.write(str(collected[17][j])+' ')
            greenq2_data.write(str(collected[18][j])+' ')
            redq2_data.write(str(collected[19][j])+' ')
            vjoint1_data.write(str(collected[20][j])+' ')
            vjoint2_data.write(str(collected[21][j])+' ')
            vjoint3_data.write(str(collected[22][j])+' ')
            vjoint4_data.write(str(collected[23][j])+' ')
            vjoint5_data.write(str(collected[24][j])+' ')
            vjoint6_data.write(str(collected[25][j])+' ')
            vjoint7_data.write(str(collected[26][j])+' ')

        # Close writable files
        joint1_data.close()
        joint2_data.close()
        joint3_data.close()
        joint4_data.close()
        joint5_data.close()
        joint6_data.close()
        joint7_data.close()
        vjoint1_data.close()
        vjoint2_data.close()
        vjoint3_data.close()
        vjoint4_data.close()
        vjoint5_data.close()
        vjoint6_data.close()
        vjoint7_data.close()
        px_data.close()
        py_data.close()
        pz_data.close()
        vx_data.close()
        vy_data.close()
        vz_data.close()
        error_data.close()
        V1_data.close()
        V2_data.close()
        greenq1_data.close()
        redq1_data.close()
        greenq2_data.close()
        redq2_data.close()"""


    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass