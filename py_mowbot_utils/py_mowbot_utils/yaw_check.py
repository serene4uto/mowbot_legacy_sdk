import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation


class YawPrinterNode(Node):
    def __init__(self):
        super().__init__('yaw_printer_node')

        # Latest values
        self.latest_global_yaw = None
        self.latest_local_yaw = None
        self.latest_gps_heading_yaw = None
        self.latest_imu_vyaw = None
        self.latest_mowbot_yaw = None
        self.latest_mowbot_vyaw = None

        # Subscribers
        self.global_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.global_odom_callback,
            10
        )

        self.local_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.local_odom_callback,
            10
        )

        self.gps_heading_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose/dual_gps_heading',
            self.gps_heading_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/mb_imu/data',
            self.imu_callback,
            10
        )

        self.mowbot_odom_sub = self.create_subscription(
            Odometry,
            '/mowbot_base/odom',
            self.mowbot_odom_callback,
            10
        )

        # Timer to print values every second
        self.timer = self.create_timer(1.0, self.print_latest_values)

    def quaternion_to_yaw(self, q):
        """
        Convert a quaternion to a yaw angle using SciPy.
        """
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler_angles = rotation.as_euler('xyz')  # Extract Euler angles (roll, pitch, yaw)
        return euler_angles[2]  # Yaw is the third element

    def global_odom_callback(self, msg):
        self.latest_global_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def local_odom_callback(self, msg):
        self.latest_local_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def gps_heading_callback(self, msg):
        self.latest_gps_heading_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def imu_callback(self, msg):
        # Extract yaw rate (vyaw) from angular velocity (z-axis)
        self.latest_imu_vyaw = msg.angular_velocity.z

    def mowbot_odom_callback(self, msg):
        # Extract yaw from pose
        self.latest_mowbot_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        # Extract yaw rate (vyaw) from twist
        self.latest_mowbot_vyaw = msg.twist.twist.angular.z

    def print_latest_values(self):
        global_yaw = f"{self.latest_global_yaw:.4f}" if self.latest_global_yaw is not None else "No data"
        local_yaw = f"{self.latest_local_yaw:.4f}" if self.latest_local_yaw is not None else "No data"
        gps_heading_yaw = f"{self.latest_gps_heading_yaw:.4f}" if self.latest_gps_heading_yaw is not None else "No data"
        imu_vyaw = f"{self.latest_imu_vyaw:.4f}" if self.latest_imu_vyaw is not None else "No data"
        # mowbot_yaw = f"{self.latest_mowbot_yaw:.4f}" if self.latest_mowbot_yaw is not None else "No data"
        mowbot_vyaw = f"{self.latest_mowbot_vyaw:.4f}" if self.latest_mowbot_vyaw is not None else "No data"

        self.get_logger().info(f"Latest Global Yaw: {global_yaw} radians")
        self.get_logger().info(f"Latest Local Yaw: {local_yaw} radians")
        self.get_logger().info(f"Latest GPS Heading Yaw: {gps_heading_yaw} radians")
        self.get_logger().info(f"Latest IMU vyaw (yaw rate): {imu_vyaw} rad/s")
        # self.get_logger().info(f"Latest Mowbot Yaw: {mowbot_yaw} radians")
        self.get_logger().info(f"Latest Mowbot vyaw (yaw rate): {mowbot_vyaw} rad/s")
        
        self.get_logger().info("------------------------------------------------------------")


def main(args=None):
    rclpy.init(args=args)
    node = YawPrinterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
