import numpy as np 
import rclpy as rospy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Float32

class Robot_Pose(Node):

    def __init__(self):

        super().__init__("Robot_Pose")

        self.declare_parameter("node_rate",1000)
        self.declare_parameter("sample_time",0.1)
        self.declare_parameter("L",0.19)
        self.declare_parameter("R",0.05)
        self.declare_parameter("cmd_vel_topic","cmd_vel")
        self.declare_parameter("chasis_pose_pub_topic","chasis_pose")
        self.declare_parameter("wr_pose_pub_topic","wr_pose")
        self.declare_parameter("wl_pose_pub_topic","wl_pose")
        
        # Assign Parameter Values
        node_rate = self.get_parameter("node_rate").get_parameter_value().integer_value 
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value 
        chasis_pose_pub_topic = self.get_parameter("chasis_pose_pub_topic").get_parameter_value().string_value 
        wl_pose_pub_topic = self.get_parameter("wl_pose_pub_topic").get_parameter_value().string_value 
        wr_pose_pub_topic = self.get_parameter("wr_pose_pub_topic").get_parameter_value().string_value 

        self.sample_time = self.get_parameter("sample_time").get_parameter_value().double_value 
        self.L = self.get_parameter("L").get_parameter_value().double_value 
        self.R = self.get_parameter("R").get_parameter_value().double_value 

        # Initialize publisers and subscribers
        self.cmd_vel_subs = self.create_subscription(Twist,cmd_vel_topic,self.read_cmd_vel,2)
        self.chasis_pose_pub = self.create_publisher(Pose, chasis_pose_pub_topic,2)
        self.wl_pose_pub = self.create_publisher(Float32, wl_pose_pub_topic,2)
        self.wr_pose_pub = self.create_publisher(Float32, wr_pose_pub_topic,2)

        self.cmd_vel = Twist()
        self.chasis_pose = Pose()
        self.wl_pose = Float32()
        self.wr_pose = Float32()

        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0
        self.first = True

        self.robot_theta = 0
        self.robot_x = 0
        self.robot_y = 0
        self.robot_z = 0
        self.wl_theta = 0
        self.wr_theta = 0

        self.timer_period = 1.0 / node_rate # hz

        self.timer = self.create_timer(self.timer_period, self.run_node)

    def read_cmd_vel(self,msg):
        
        self.cmd_vel.linear.x = msg.linear.x 
        self.cmd_vel.linear.y = msg.linear.y 
        self.cmd_vel.linear.z = msg.linear.z 

        self.cmd_vel.angular.x = msg.angular.x 
        self.cmd_vel.angular.y = msg.angular.y 
        self.cmd_vel.angular.z = msg.angular.z 

    def get_pose(self,dt):

        self.robot_theta += self.cmd_vel.angular.z * dt 
        self.robot_x += (self.cmd_vel.linear.x * dt) * np.cos(self.robot_theta)
        self.robot_y += (self.cmd_vel.linear.x * dt) * np.sin(self.robot_theta)

        wr = (self.cmd_vel.linear.x - self.cmd_vel.angular.z * self.L /2.0) / self.R
        wl = (self.cmd_vel.linear.x + self.cmd_vel.angular.z * self.L /2.0) / self.R
        
        self.wr_theta += wr * dt
        self.wl_theta += wl * dt
    
    def run_node(self):
    
        if self.first == True:
            self.start_time = self.get_clock().now().nanoseconds
            self.current_time = self.get_clock().now().nanoseconds
            self.last_time = self.get_clock().now().nanoseconds
            self.first = False
        else:
            self.current_time = self.get_clock().now().nanoseconds
            dt = self.current_time - self.last_time
            # Scale time delta to s
            dt = dt * (10 ** (-9))
            if dt >= self.sample_time:
                print("dt : " + str(dt))
                self.get_pose(dt)
                
                self.chasis_pose.position.x = self.robot_x
                self.chasis_pose.position.y = self.robot_y
                self.chasis_pose.orientation.z = self.robot_theta

                self.wr_pose.data = self.wr_theta
                self.wl_pose.data = self.wl_theta

                self.chasis_pose_pub.publish(self.chasis_pose)
                self.wl_pose_pub.publish(self.wl_pose)
                self.wr_pose_pub.publish(self.wr_pose)

                self.last_time = self.get_clock().now().nanoseconds
            

def main(args=None):
    rospy.init(args=args)

    manager = Robot_Pose()

    print("Runnig Robot_Pose ")

    rospy.spin(manager)

    manager.destroy_node()

    rospy.shutdown()


if __name__ == '__main__':
    main()
    