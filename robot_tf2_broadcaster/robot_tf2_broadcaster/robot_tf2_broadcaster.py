from math import sin, cos, pi
import rclpy as rospy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        self.declare_parameter("chasis_pose_pub_topic","chasis_pose")
        self.declare_parameter("joint_states_pub_topic","joint_states")
        self.declare_parameter("wr_pose_pub_topic","wr_pose")
        self.declare_parameter("wl_pose_pub_topic","wl_pose")
        self.declare_parameter("world_reference_frame","base_footprint")
        self.declare_parameter("robot_chasis_frame","chasis")
        self.declare_parameter("base_joint_name","base_joint")
        self.declare_parameter("base_to_left_wheel_name","base_to_left_wheel")
        self.declare_parameter("base_to_right_wheel_name","base_to_right_wheel")

        joint_states_pub_topic = self.get_parameter("joint_states_pub_topic").get_parameter_value().string_value 
        chasis_pose_pub_topic = self.get_parameter("chasis_pose_pub_topic").get_parameter_value().string_value 
        wl_pose_pub_topic = self.get_parameter("wl_pose_pub_topic").get_parameter_value().string_value 
        wr_pose_pub_topic = self.get_parameter("wr_pose_pub_topic").get_parameter_value().string_value 

        self.world_reference_frame = self.get_parameter("world_reference_frame").get_parameter_value().string_value 
        self.robot_chasis_frame = self.get_parameter("robot_chasis_frame").get_parameter_value().string_value 
        self.base_joint_name = self.get_parameter("base_joint_name").get_parameter_value().string_value 
        self.base_to_left_wheel_name = self.get_parameter("base_to_left_wheel_name").get_parameter_value().string_value 
        self.base_to_right_wheel_name = self.get_parameter("base_to_right_wheel_name").get_parameter_value().string_value 
        

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, joint_states_pub_topic, qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()

        self.wl_theta = 0.
        self.wr_theta = 0.

        self.subscription = self.create_subscription(
            Pose,
            chasis_pose_pub_topic,
            self.handle_pose,
            1)
        self.subscription  # prevent unused variable warning

        self.wl_subscription = self.create_subscription(
            Float32,
            wl_pose_pub_topic,
            self.read_wl,
            1)

        self.wr_subscription = self.create_subscription(
            Float32,
            wr_pose_pub_topic,
            self.read_wr,
            1)

    def read_wl(self,msg):
        self.wl_theta = msg.data
        # print(self.wl_theta)
    
    def read_wr(self,msg):
        self.wr_theta = msg.data
        # print(self.wl_theta)

    def handle_pose(self,msg):
         # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = self.world_reference_frame
        self.odom_trans.child_frame_id = self.robot_chasis_frame

        self.joint_state = JointState()
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [self.base_joint_name,self.base_to_left_wheel_name ,self.base_to_right_wheel_name ]
        self.joint_state.position = [0.,-self.wl_theta,-self.wr_theta]
        self.joint_pub.publish(self.joint_state)
        # update transform
        # (moving in a circle with radius=2)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = 0. - msg.position.y
        self.odom_trans.transform.translation.y = 0.+ msg.position.x
        self.odom_trans.transform.translation.z = 0.
        self.odom_trans.transform.rotation = \
        euler_to_quaternion(0., 0., 1.57 + msg.orientation.z) # roll,pitch,yaw

        # send the joint state and transform
        self.broadcaster.sendTransform(self.odom_trans)  
        # self.broadcaster.sendTransform(self.wheel_rotate)  

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    # print(f'x:{qx}, y:{qy}, z:{qz}, w:{qw}')
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rospy.init(args=args)

    manager = StatePublisher()

    print("Runnig StatePublisher ")

    rospy.spin(manager)

    manager.destroy_node()

    rospy.shutdown()


if __name__ == '__main__':
    main()