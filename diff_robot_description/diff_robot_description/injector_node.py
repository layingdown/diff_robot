import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovarianceInjector(Node):
    def __init__(self):
        super().__init__('odom_covariance_injector')

        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.pub = self.create_publisher(
            Odometry, '/odom/with_covariance', 10)

        self.pose_cov = [0.0]*36
        self.pose_cov[0]  = 0.02
        self.pose_cov[7]  = 0.02
        self.pose_cov[14] = 1e6
        self.pose_cov[21] = 1e6
        self.pose_cov[28] = 1e6
        self.pose_cov[35] = 0.05

        self.twist_cov = [0.0]*36
        self.twist_cov[0]  = 0.02
        self.twist_cov[7]  = 1e6
        self.twist_cov[14] = 1e6
        self.twist_cov[21] = 1e6
        self.twist_cov[28] = 1e6
        self.twist_cov[35] = 0.05

    def odom_callback(self, msg: Odometry):
        msg.pose.covariance = self.pose_cov
        msg.twist.covariance = self.twist_cov
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
