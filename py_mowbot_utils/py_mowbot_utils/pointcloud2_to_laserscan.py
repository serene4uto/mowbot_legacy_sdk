import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np

class PointCloud2ToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud2_to_laserscan')
        
        # Parameters (you may want to make these configurable)
        self.angle_min = -np.pi        # Start angle [-pi]
        self.angle_max = np.pi         # End angle [pi]
        self.angle_increment = np.pi / 180  # Angular resolution [1 degree]
        self.range_min = 0.0           # Minimum range value
        self.range_max = 30.0          # Maximum range value
        self.scan_time = 0.1           # Time between scans [seconds]
        self.min_height = 0.2         # Minimum height (z-axis)
        self.max_height = 1.0          # Maximum height (z-axis)

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            '/roboscanPointCloud',       # Replace with your PointCloud2 topic
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(
            LaserScan,
            '/pointcloud2_to_laserscan/roboscanLaserScan',  # Replace with your LaserScan topic
            10)

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to a structured array
        points_gen = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.array(list(points_gen), dtype=[
                          ('x', np.float32), ('y', np.float32), ('z', np.float32)])
        
        # Check if there are points to process
        if points.size == 0:
            self.get_logger().warn("Received an empty point cloud.")
            return

        # Process the points to generate LaserScan data
        scan_ranges = self.pointcloud_to_laserscan(points)

        # Create and publish the LaserScan message
        laser_scan = LaserScan()
        laser_scan.header = msg.header
        laser_scan.angle_min = self.angle_min
        laser_scan.angle_max = self.angle_max
        laser_scan.angle_increment = self.angle_increment
        laser_scan.time_increment = 0.0  # Not used in this example
        laser_scan.scan_time = self.scan_time
        laser_scan.range_min = self.range_min
        laser_scan.range_max = self.range_max
        laser_scan.ranges = scan_ranges.tolist()
        
        self.publisher.publish(laser_scan)

    def pointcloud_to_laserscan(self, points):
        # Extract x, y, and z coordinates from the structured array
        x = points['x']
        y = points['y']
        z = points['z']

        # Filter points based on min_height and max_height
        height_mask = (z >= self.min_height) & (z <= self.max_height)
        x = x[height_mask]
        y = y[height_mask]

        # Check if there are points to process after filtering
        if x.size == 0:
            self.get_logger().warn("No points within the specified height range.")
            return np.array([])

        # Compute the angle and range for each point
        angles = np.arctan2(y, x)
        ranges = np.hypot(x, y)

        # Initialize the scan ranges with infinity
        num_ranges = int(
            np.ceil((self.angle_max - self.angle_min) / self.angle_increment))
        scan_ranges = np.full(num_ranges, np.inf)

        # Bin the points into the appropriate angle increments
        for angle, range_ in zip(angles, ranges):
            if self.range_min < range_ < self.range_max:
                # Find the corresponding bin index
                bin_index = int((angle - self.angle_min) / self.angle_increment)
                if 0 <= bin_index < num_ranges:
                    # Keep the smallest range (closest obstacle) for each bin
                    if range_ < scan_ranges[bin_index]:
                        scan_ranges[bin_index] = range_

        # Replace infinities with range_max + 1.0 to indicate no detection within range
        scan_ranges = np.where(
            np.isinf(scan_ranges), self.range_max + 1.0, scan_ranges)

        return scan_ranges

def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2ToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
