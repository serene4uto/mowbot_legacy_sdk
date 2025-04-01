import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from rtcm_msgs.msg import Message as Rtcm
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__('sensor_monitor_node')

        self.last_received_times = {
            '/imu/data': None,
            '/scan': None,
            '/rtcm': None,
            '/imu_gps_heading/data': None,
            '/ublox_gpsl_node/fix': None,
            '/ublox_gpsr_node/fix': None,
            '/mowbot_base/odom': None,
        }

        # Subscriptions for the topics
        self.create_subscription(Imu, '/imu/data', self.generic_callback('/imu/data'), 10)
        self.create_subscription(LaserScan, '/scan', self.generic_callback('/scan'), 10)
        self.create_subscription(Rtcm, '/rtcm', self.generic_callback('/rtcm'), 10)
        self.create_subscription(Imu, '/imu_gps_heading/data', self.generic_callback('/imu_gps_heading/data'), 10)
        self.create_subscription(NavSatFix, '/ublox_gpsl_node/fix', self.generic_callback('/ublox_gpsl_node/fix'), 10)
        self.create_subscription(NavSatFix, '/ublox_gpsr_node/fix', self.generic_callback('/ublox_gpsr_node/fix'), 10)
        self.create_subscription(Odometry, '/mowbot_base/odom', self.generic_callback('/mowbot_base/odom'), 10)

        # Publisher for sensor status
        self.status_publisher = self.create_publisher(DiagnosticArray, '/sensor_status', 10)

        # Timer to check sensor status periodically
        self.create_timer(1.0, self.check_sensor_status)

    def generic_callback(self, topic):
        def callback(msg):
            self.last_received_times[topic] = self.get_clock().now()
        return callback

    def check_sensor_status(self):
        now = self.get_clock().now()
        diag_array = DiagnosticArray()
        diag_array.header.stamp = now.to_msg()
        diag_array.header.frame_id = "sensor_monitor_frame"

        topic_aliases = {
            '/imu/data': 'IMU',
            '/scan': 'Lidar',
            '/rtcm': 'RTCM',
            '/imu_gps_heading/data': 'Heading',
            '/ublox_gpsl_node/fix': 'Left GPS',
            '/ublox_gpsr_node/fix': 'Right GPS',
            '/mowbot_base/odom': 'AMR Base',
        }

        for topic, last_time in self.last_received_times.items():
            alias = topic_aliases.get(topic, topic)
            status = DiagnosticStatus()
            status.name = alias
            status.hardware_id = "sensor_monitor"
            if last_time is None or (now - last_time).nanoseconds / 1e9 >= 2.0:
                status.level = DiagnosticStatus.WARN  # Inactive sensors get WARN
                status.message = "Inactive"
                self.get_logger().warn(f"{alias}: Inactive")
            else:
                status.level = DiagnosticStatus.OK  # Active sensors get OK
                status.message = "Active"
                self.get_logger().info(f"{alias}: Active")

            diag_array.status.append(status)

        # Publish the diagnostic array
        self.status_publisher.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    
    sensor_monitor_node = SensorMonitorNode()

    try:
        rclpy.spin(sensor_monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
