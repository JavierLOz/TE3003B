#!/usr/bin/env python
import rospy
import numpy as np
from robot_pose.msg import r_pose
from std_msgs.msg import Float32

class Robot_Pose:

    def __init__(self):

        self.wl = Float32()
        self.wl.data = 0.0

        self.wr = Float32()
        self.wr.data = 0.0
        
        self.pose = r_pose()
        self.pose.x = 0.0
        self.pose.y = 0.0
        
        self.pose.theta = np.pi/2

        self.star_time = 0
        self.current_time = 0  
        self.last_time = 0
        self.sample_time = 0

        self.first = True

        self.robot_r = 0.05 #rospy.get_param("/robot_pose_r",0.05)
        self.robot_l = 0.19 #rospy.get_param("/robot_pose_l",0.19)
        self.sample_time = 0.02 #rospy.get_param("/robot_pose_dt",0.02)


        #Setup Publishers and subscribers here
        rospy.Subscriber("wl",Float32,self.callback_wl)
        rospy.Subscriber("wr",Float32,self.callback_wr)
        self.posePub = rospy.Publisher("robot_Pose",r_pose,queue_size=10)

    def callback_wl(self,msg):
   
        self.wl = msg
    
    def callback_wr(self,msg):
        self.wr = msg
    

    def update_pose(self,dt):
        
        self.pose.x +=  ((self.robot_r / 2) * (self.wl.data + self.wr.data) * dt) * np.cos(self.pose.theta)
        self.pose.y +=  ((self.robot_r / 2) * (self.wl.data + self.wr.data) * dt) * np.sin(self.pose.theta)
        self.pose.theta += ((self.robot_r /self.robot_l) * (self.wr.data - self.wl.data) * dt)

        if self.pose.theta >= (np.pi):
            self.pose.theta = (self.pose.theta - (2 * np.pi))
    
    def run_node(self):
        if self.first == True:
            self.star_time = rospy.get_time()
            self.current_time = rospy.get_time()
            self.last_time = rospy.get_time()
            self.first = False
        else:
            self.current_time = rospy.get_time()
            dt = self.current_time - self.last_time

            if dt >= self.sample_time:
                #print("dt : " + str(dt))
                self.update_pose(dt)
                self.posePub.publish(self.pose)
                self.last_time = rospy.get_time()

    def stop(self):
        #Setup the stop message (can be the same as the control message)
        print("Stopping")


if __name__=='__main__':
    
    #Initialise and Setup node
    rospy.init_node("robot_pose")
    
    nodeRate = rospy.get_param("/robot_pose_nodeRate",200)
    rate = rospy.Rate(nodeRate)
    rate.sleep()

    robotPose = Robot_Pose()

    rospy.on_shutdown(robotPose.stop)

  
    print("The Controller is Running")
    
    while not rospy.is_shutdown():
      robotPose.run_node()
      rate.sleep()


    