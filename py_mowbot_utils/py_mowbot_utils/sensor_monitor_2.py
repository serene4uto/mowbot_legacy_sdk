import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from rtcm_msgs.msg import Message as Rtcm
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class SensorMonitorNode(Node):
    # Constants to avoid magic numbers
    INACTIVE_THRESHOLD_SEC = 2.0
    UPDATE_INTERVAL_SEC = 1.0
    QOS_DEPTH = 10

    def __init__(self):
        super().__init__('sensor_monitor_node')
        
        # Define topic configurations with topic name, message type, and alias
        self.topic_configs = {
            '/imu/data': {'type': Imu, 'alias': 'IMU'},
            '/scan': {'type': LaserScan, 'alias': 'Lidar'},
            '/rtcm': {'type': Rtcm, 'alias': 'RTCM'},
            '/gps/heading': {'type': Imu, 'alias': 'Heading'},
            '/gps/fix': {'type': NavSatFix, 'alias': 'GPS'},
            '/mowbot_base/odom': {'type': Odometry, 'alias': 'AMR Base'},
        }
        
        # Initialize last_received_times with monitored topics
        self.last_received_times = {
            topic: None for topic in self.topic_configs
            if topic != '/mowbot_base/odom'  # Exclude odom if not needed
        }
        
        # For tracking status changes to reduce logging
        self._prev_statuses = {}
        
        # Create subscriptions dynamically
        for topic, config in self.topic_configs.items():
            if topic in self.last_received_times:  # Only subscribe to monitored topics
                self.create_subscription(
                    config['type'],
                    topic,
                    self.generic_callback(topic),
                    self.QOS_DEPTH
                )

        # Publisher for sensor status
        self.status_publisher = self.create_publisher(
            DiagnosticArray, '/sensor_status', self.QOS_DEPTH)

        # Timer to check sensor status periodically
        self.create_timer(self.UPDATE_INTERVAL_SEC, self.check_sensor_status)

    def generic_callback(self, topic):
        def callback(msg):
            try:
                self.last_received_times[topic] = self.get_clock().now()
            except Exception as e:
                self.get_logger().error(f"Error in callback for {topic}: {str(e)}")
        return callback

    def check_sensor_status(self):
        try:
            now = self.get_clock().now()
            diag_array = DiagnosticArray()
            diag_array.header.stamp = now.to_msg()
            diag_array.header.frame_id = "sensor_monitor_frame"
            
            for topic, last_time in self.last_received_times.items():
                config = self.topic_configs.get(topic, {})
                alias = config.get('alias', topic)
                
                status = DiagnosticStatus()
                status.name = alias
                status.hardware_id = "mowbot"
                
                is_active = (
                    last_time is not None and 
                    (now - last_time).nanoseconds / 1e9 < self.INACTIVE_THRESHOLD_SEC
                )
                current_status = "Active" if is_active else "Inactive"
                status.level = DiagnosticStatus.OK if is_active else DiagnosticStatus.WARN
                status.message = current_status
                
                # Only log if status has changed from previous check
                if topic not in self._prev_statuses or self._prev_statuses[topic] != current_status:
                    # Fix: Use consistent logging method and format the message with appropriate prefix
                    if is_active:
                        self.get_logger().info(f"{alias}: {current_status}")
                    else:
                        self.get_logger().warn(f"{alias}: {current_status}")
                
                self._prev_statuses[topic] = current_status
                diag_array.status.append(status)
            
            # Publish the diagnostic array
            self.status_publisher.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Error in check_sensor_status: {str(e)}")


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