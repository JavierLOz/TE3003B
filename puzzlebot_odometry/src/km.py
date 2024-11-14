#Implementation of EKF for PuzzleBot Localization
#Author: Arturo E. Ceron-Lopez @ ITESM Campus MTY (2024)
#License: Not for commercial use, author acknowledgement required

import math
import time
import copy
import numpy as np

import rospy
from std_msgs.msg import Float32, Time, Header, Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf.transformations
import tf_conversions
import tf2_ros

class EKFLocalization:
    def __init__(self):
        # Robot Properties
        self.dt = 0.01 # Delta Time
        self.prev_time = 0.0
        self.current_time = 0.0
        self.start_time = 0.0

        self.r = 0.05 #  wheel radius [meters]
        self.l = 0.19 #  wheel distance [meters]

        # Robot State
        self.s = np.array([0.0, 0.0, 0.0]) # Current Position State Vector (x [meters], y [meters], theta [radians])
        self.prev_s = np.array([0.0, 0.0, 0.0]) # Previous Position State Vector (x [meters], y [meters], theta [radians])
        
        self.wl = 0.0 # Current Left Wheel angular velocity [rad/sec]
        self.wr = 0.0 # Current Right Wheel angular velocity [rad/sec]

        self.v_k = 0.0 # Desired Robot Linear Velocity in X axis [meters/sec]
        self.w_k = 0.0 # Desired Robot Angular Velocity in Z axis [rad/sec]

        self.H = np.array([[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0], 
                          [0.0, 0.0, 1.0]]) # Robot Pose Model Jacobian Matrix

        self.Sigma = np.array([[0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0], 
                          [0.0, 0.0, 0.0]]) # Current Robot Pose Model Covariance Matrix
        
        self.prev_Sigma = np.array([[0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0], 
                          [0.0, 0.0, 0.0]]) # Previous Robot Pose Model Covariance Matrix

        self.Q = np.array([[0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0], 
                          [0.0, 0.0, 0.0]]) # Current Robot Pose Noise Covariance Matrix
        
        self.kl = 0.15 # Noise tuning parameter for Left Wheel
        self.kr = 0.15 # Noise tuning parameter for Right Wheel

        self.SigmaD = np.array([[0.0, 0.0],
                               [0.0, 0.0]]) # Wheel Noise Covariance Matrix

        self.Nabla = np.array([[0.0, 0.0],
                              [0.0, 0.0],
                              [2.0/self.l, 2.0/self.l]]) # Wheel Model Noise Jacobian Matrix

        # Sensor State
        self.r_rho = 0.0
        self.r_alpha =0.0

        self.z_rho = 0.0
        self.z_alpha = 0.0
        
        self.z_measured = np.array([0.0, 0.0]) # Measured Robot Pose Observation
        self.z_static = np.array([0.0, 0.0])

        self.delta_zx = 0.0 # Helper
        self.delta_yx = 0.0 # Helper
        self.p = 0.0 # Helper

        self.z = np.transpose(np.array([0.0, 0.0])) # Corrected Robot Pose Observation

        self.G = np.array([[0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0]]) # Current Robot Pose Observation Jacobian Matrix

        self.Z = np.array([[0.0, 0.0],
                          [0.0, 0.0]]) # Robot Pose Observation Covariance Matrix

        self.R = np.array([[0.0, 0.0],
                          [0.0, 0.0]]) # Current Robot Observation Noise Covariance Matrix
        
        self.K = np.array([[0.0, 0.0],
                          [0.0, 0.0], 
                          [0.0, 0.0]]) # Kalman Gain
        
        # ROS messages
        self.odometry = Odometry()
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = 'world'
        self.odometry.child_frame_id = 'odom'

        self.odom_trans = TransformStamped()
        self.odom_trans.header.stamp = rospy.Time.now()
        self.odom_trans.header.frame_id = 'world'
        self.odom_trans.child_frame_id = 'odom'

        # ROS Node Publishers
        self.odometry_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def wrapToPi(self, theta):
        if theta > np.pi:
                theta = theta - 2*np.pi
        elif theta < -np.pi:
            theta = theta + 2*np.pi
        return theta
    
    def Calc_deltaT(self):
        self.current_time = time.time() - self.start_time
        self.dt = self.current_time - self.prev_time

    def Calc_Gradient_w(self):
        self.Nabla[0, 0] = math.cos(self.prev_s[2])
        self.Nabla[0, 1] = math.cos(self.prev_s[2])
        self.Nabla[1, 0] = math.sin(self.prev_s[2])
        self.Nabla[1, 1] = math.sin(self.prev_s[2])
        self.Nabla = 0.5*self.r*self.dt*self.Nabla
    
    def Calc_SigmaD(self):
        self.SigmaD[0, 0] = self.kr*abs(self.wr)
        self.SigmaD[1, 1] = self.kl*abs(self.wl)

    def Estimate_Q(self):
        self.Calc_Gradient_w()
        self.Calc_SigmaD()
        self.Q = np.dot(np.dot(self.Nabla, self.SigmaD), np.transpose(self.Nabla))
    
    def Calc_miuHat(self):
        self.s[0] = self.prev_s[0] + self.dt*self.v_k*math.cos(self.prev_s[2])
        self.s[1] = self.prev_s[1]  + self.dt*self.v_k*math.sin(self.prev_s[2])
        self.s[2] = self.prev_s[2]  + self.dt*self.w_k
        
        print("miuHat:")
        print(self.s)
        print("****")

    def Calc_Gradient_h(self):
        self.H[0, 2] = -self.dt*self.v_k*math.sin(self.prev_s[2])
        self.H[1, 2] = self.dt*self.v_k*math.cos(self.prev_s[2])
        print("H:")
        print(self.H)
        print("****")

    def Calc_SigmaHat(self):
        self.Sigma = np.dot(np.dot(self.H, self.prev_Sigma), np.transpose(self.H)) + self.Q
        print("SigmaHat:")
        print(self.Sigma)
        print("****")

    def Calc_zHat(self):
        self.delta_zx = self.z_static[0] - self.s[0]
        self.delta_zy = self.z_static[1] - self.s[1]
        self.p = self.delta_zx**2 + self.delta_zy**2
        print("zDelta,p:")
        print(self.delta_zx)
        print(self.delta_zy)
        print(self.p)
        print("****")

        self.z[0] = np.sqrt(self.p)
        self.z[1] = self.wrapToPi(np.arctan2(self.delta_zy, self.delta_zx) - self.s[2])
        print("zHat:")
        print(self.z)
        print("****")

    def Calc_Gradient_g(self):
        self.G[0, 0] = -self.delta_zx/np.sqrt(self.p)
        self.G[0, 1] = -self.delta_zy/np.sqrt(self.p)
        self.G[0, 2] = 0.0
        self.G[1, 0] = self.delta_zy/self.p
        self.G[1, 1] = -self.delta_zx/self.p
        self.G[1, 2] = -1.0
        print("G:")
        print(self.G)
        print("****")

    def Calc_Z(self):
        self.Z = np.dot(np.dot(self.G, self.Sigma), np.transpose(self.G)) + self.R
        print("Z:")
        print(self.Z)
        print("****")

    def Calc_KalmanGain(self):
        self.K = np.dot(np.dot(self.Sigma, np.transpose(self.G)), np.linalg.inv(self.Z))
        print("K:")
        print(self.K)
        print("****")

    def Calc_miu(self):
        self.s = self.s + np.dot(self.K, self.z_measured - self.z)
        print("miu:")
        print(self.s)
        print("****")

    def Calc_Sigma(self):
        self.Sigma = np.dot(np.eye(3) - np.dot(self.K, self.G), self.Sigma)
        print("Sigma:")
        print(self.Sigma)
        print("****")

    def PublishResult(self):
        s_copy = copy.deepcopy(self.s)
        Sigma_copy = copy.deepcopy(self.Sigma)
        u_v_copy = copy.copy(self.v_k)
        u_w_copy = copy.copy(self.w_k)

        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.pose.pose.position.x = s_copy[0]
        self.odometry.pose.pose.position.y = s_copy[1]
        self.odometry.pose.pose.position.z = 0.0

        euler2quat = tf_conversions.transformations.quaternion_from_euler(0, 0, s_copy[2])
        self.odometry.pose.pose.orientation.x = euler2quat[0]
        self.odometry.pose.pose.orientation.y = euler2quat[1]
        self.odometry.pose.pose.orientation.z = euler2quat[2]
        self.odometry.pose.pose.orientation.w = euler2quat[3]

        self.odometry.twist.twist.linear.x = u_v_copy
        self.odometry.twist.twist.angular.z = u_w_copy

        self.odometry.pose.covariance[0]  = Sigma_copy[0, 0]
        self.odometry.pose.covariance[1]  = Sigma_copy[0, 1]
        self.odometry.pose.covariance[5]  = Sigma_copy[0, 2]
        self.odometry.pose.covariance[6]  = Sigma_copy[1, 0]
        self.odometry.pose.covariance[7]  = Sigma_copy[1, 1]
        self.odometry.pose.covariance[11] = Sigma_copy[1, 2]
        self.odometry.pose.covariance[30] = Sigma_copy[2, 0]
        self.odometry.pose.covariance[31] = Sigma_copy[2, 1]
        self.odometry.pose.covariance[35] = Sigma_copy[2, 2]

        self.odom_trans.header.stamp = copy.deepcopy(self.odometry.header.stamp)
        self.odom_trans.transform.translation.x = s_copy[0]
        self.odom_trans.transform.translation.y = s_copy[1]
        self.odom_trans.transform.translation.z = 0.0

        self.odom_trans.transform.rotation.x = euler2quat[0]
        self.odom_trans.transform.rotation.y = euler2quat[1]
        self.odom_trans.transform.rotation.z = euler2quat[2]
        self.odom_trans.transform.rotation.w = euler2quat[3]

        self.tf_broadcaster.sendTransform(self.odom_trans)
        self.odometry_pub.publish(self.odometry)

        print("Odometry:")
        print(self.odometry)
        print("****")

    def Set_Previous(self):
        self.prev_s = copy.deepcopy(self.s)
        self.prev_Sigma = copy.deepcopy(self.Sigma)
        self.prev_time = self.current_time

    def PoseEstimation(self, start_t, landmark_measured, landmark_pos, landmark_status, control_u, measured_wl, measured_wr, calibrated_klr, calibrated_R):
        self.z_measured = np.transpose(landmark_measured)
        self.z_static = np.transpose(landmark_pos)

        self.v_k = control_u[0]
        self.w_k = control_u[1]

        self.wl = measured_wl
        self.wr = measured_wr

        self.kl = calibrated_klr[0]
        self.kr = calibrated_klr[1]

        self.R = calibrated_R

        self.start_time = start_t

        self.Calc_deltaT()
        #self.dt = 0.1 #

        self.Estimate_Q()
        self.Q = np.array([[0.5, 0.01, 0.01],
                          [0.01, 0.5, 0.01],
                          [0.01, 0.01, 0.2]])
        
        self.Calc_miuHat() # Ideal calculated pose
        self.Calc_Gradient_h() # Pose Model linerization
        self.Calc_SigmaHat() # Uncertainty of Ideal calculated pose
        if (landmark_status):
            self.Calc_zHat() # Ideal observed pose
            self.Calc_Gradient_g() # Observation Model linerization
            self.Calc_Z() # Uncertainty of Ideal observed pose
            self.Calc_KalmanGain() # Kalman Filter Gain!
            self.Calc_miu() # Estimated pose
            self.Calc_Sigma() # Estimated uncertainty of pose
        self.Set_Previous()

if __name__=='__main__':
    try:
        rospy.init_node("kalman_node")
        rate = rospy.Rate(1)

        kalman_test = EKFLocalization()

        mark_real_pos = np.array([3, 4])
        R_calib = np.array([[0.1, 0],[0, 0.2]])
        mark_measurements = np.array([[4.87, 0.8], [4.72, 0.72], [4.69, 0.65], [4.49, 0.55], [4.29, 0.45], [4.0, 0.3]])

        kalman_test.PublishResult()
        rate.sleep()
        kalman_test.PublishResult()
        rate.sleep()

        start_t = time.time()
        for i in range(len(mark_measurements)):
            print("----" + str(i) + "----")
            kalman_test.PoseEstimation(start_t, mark_measurements[i,:], mark_real_pos, True, np.array([1.0, 1.0]), 0.0, 0.0, np.array([0.0, 0.0]), R_calib)
            kalman_test.PublishResult()
            rate.sleep()

    except rospy.ROSInterruptException:
        print("end")
        exit(0)