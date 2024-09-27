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

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()

        self.wl_theta = 0.
        self.wr_theta = 0.

        self.subscription = self.create_subscription(
            Pose,
            'chasis_pose',
            self.handle_pose,
            1)
        self.subscription  # prevent unused variable warning

        self.wl_subscription = self.create_subscription(
            Float32,
            'wl_pose',
            self.read_wl,
            1)

        self.wr_subscription = self.create_subscription(
            Float32,
            'wr_pose',
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
        self.odom_trans.header.frame_id = 'base_footprint'
        self.odom_trans.child_frame_id = 'chasis'

        self.joint_state = JointState()
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['base_joint','base_to_left_wheel','base_to_right_wheel']
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