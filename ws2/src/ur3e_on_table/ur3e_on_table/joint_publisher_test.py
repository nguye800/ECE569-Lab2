import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class JointPublisherTest(Node):

    def __init__(self):
        super().__init__('joint_publisher_test')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.t = 0

    def timer_callback(self):
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        msg.position = [2*np.pi*self.t/5, -1+0.5*np.cos(2*np.pi*self.t/10), np.sin(2*np.pi*self.t/15), 0.0, 0.0, 0.0]
        msg.velocity = []
        msg.effort = []
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing position: "{msg.position}"')
        
        self.t += self.timer_period


def main(args=None):
    rclpy.init(args=args)

    joint_publisher_test_node = JointPublisherTest()

    rclpy.spin(joint_publisher_test_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_publisher_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()