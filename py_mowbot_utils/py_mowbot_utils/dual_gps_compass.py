import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PointStamped

import tf2_geometry_msgs
import tf2_ros

from scipy.spatial.transform import Rotation as R
import pyproj
import numpy as np

def get_utm_coordinates(latitude, longitude):
    # Define the source (WGS84) and target (UTM) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu, South Korea)
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(wgs84, crs, always_xy=True)
    utm_easting, utm_northing = transformer.transform(longitude, latitude)
    return utm_easting, utm_northing

def get_wgs84_coordinates(utm_easting, utm_northing):
    # Define the source (UTM) and target (WGS84) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")
    crs = pyproj.CRS("EPSG:32652")
    transformer = pyproj.Transformer.from_crs(crs, wgs84, always_xy=True)
    longitude, latitude = transformer.transform(utm_easting, utm_northing)
    return latitude, longitude

class DualGpsCompass(Node):

    def __init__(self):
        super().__init__('dual_gps_compass')

        self.gps_left_frame_id = 'gps_left_link'
        self.gps_right_frame_id = 'gps_right_link'
        self.base_frame_id = 'base_link'
        self.combined_gps_frame_id = 'combined_gps_link'
        self.publish_combined_gps_tf = True

        self.gps_left_updated = False
        self.gps_right_updated = False

        self.last_gps_left = None
        self.last_gps_right = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.gps_left_sub = self.create_subscription(
            NavSatFix, '/ublox_gpsl_node/fix', self.gps_left_callback, 10)
        self.gps_right_sub = self.create_subscription(
            NavSatFix, '/ublox_gpsr_node/fix', self.gps_right_callback, 10)

        self.gps_fix_pub = self.create_publisher(NavSatFix, '/combined_gps/fix', 10)
        # self.dual_gps_heading_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose/dual_gps_heading', 10)
        # self.dual_gps_heading_pub = self.create_publisher(Odometry, '/odom/dual_gps_heading', 10)
        
        self.heading_process_timer = self.create_timer(0.05, self.heading_process_timer_callback)

        self.imu_gps_heading_pub = self.create_publisher(Imu, '/imu_gps_heading/data', 10)

    def gps_left_callback(self, msg: NavSatFix):
        # Check if the GPS data is valid
        if msg.status.status == -1:
            self.get_logger().info('Invalid GPS data from left GPS')
            return

        self.last_gps_left = msg
        self.gps_left_updated = True

    def gps_right_callback(self, msg: NavSatFix):
        # Check if the GPS data is valid
        if msg.status.status == -1:
            self.get_logger().info('Invalid GPS data from right GPS')
            return

        self.last_gps_right = msg
        self.gps_right_updated = True

    def heading_process_timer_callback(self):
        # Ensure both GPS data are updated
        if not self.gps_left_updated or not self.gps_right_updated:
            self.get_logger().info('Waiting for both GPS data')
            return

        combined_gps_altitude = (self.last_gps_left.altitude + self.last_gps_right.altitude) / 2

        # Get the UTM coordinates
        utm_left_x, utm_left_y = get_utm_coordinates(
            self.last_gps_left.latitude, self.last_gps_left.longitude)
        utm_right_x, utm_right_y = get_utm_coordinates(
            self.last_gps_right.latitude, self.last_gps_right.longitude)

        # Calculate the relative position
        relative_x = utm_right_x - utm_left_x
        relative_y = utm_right_y - utm_left_y

        # # Adjust the relative positions to align with your robot's coordinate frame
        # # Swap axes and invert one axis based on your robot's orientation
        # adjusted_relative_x = relative_y  # Swap axes
        # adjusted_relative_y = -relative_x  # Invert one axis

        # # Determine the heading angle
        # heading_yaw = np.arctan2(adjusted_relative_y, adjusted_relative_x)  # In radians
        
        # Determine the heading angle
        heading_yaw = np.arctan2(relative_y, relative_x) + np.pi / 2  # Adjust by 90 degrees to align with ENU frame
        # Normalize heading_yaw to be between -π and π
        heading_yaw = (heading_yaw + np.pi) % (2 * np.pi) - np.pi

        # Get transforms from gps_left_link and gps_right_link to base_link
        # try:
        #     transform_left = self.tf_buffer.lookup_transform(
        #         self.base_frame_id, self.gps_left_frame_id, rclpy.time.Time())
        #     transform_right = self.tf_buffer.lookup_transform(
        #         self.base_frame_id, self.gps_right_frame_id, rclpy.time.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     self.get_logger().error(f'Error: {e}')
        #     return

        # # Use a reference point to reduce the magnitude of UTM coordinates
        # ref_utm_x = (utm_left_x + utm_right_x) / 2
        # ref_utm_y = (utm_left_y + utm_right_y) / 2

        # # Compute relative positions with respect to the reference point
        # utm_left_x_rel = utm_left_x - ref_utm_x
        # utm_left_y_rel = utm_left_y - ref_utm_y
        # utm_right_x_rel = utm_right_x - ref_utm_x
        # utm_right_y_rel = utm_right_y - ref_utm_y

        # # Create PointStamped messages with relative positions
        # point_left = PointStamped()
        # point_left.header.stamp = self.get_clock().now().to_msg()
        # point_left.header.frame_id = 'utm'
        # point_left.point.x = utm_left_x_rel
        # point_left.point.y = utm_left_y_rel
        # point_left.point.z = 0.0

        # point_right = PointStamped()
        # point_right.header.stamp = self.get_clock().now().to_msg()
        # point_right.header.frame_id = 'utm'
        # point_right.point.x = utm_right_x_rel
        # point_right.point.y = utm_right_y_rel
        # point_right.point.z = 0.0

        # # Transform the points into the base frame
        # base_point_left = tf2_geometry_msgs.do_transform_point(point_left, transform_left)
        # base_point_right = tf2_geometry_msgs.do_transform_point(point_right, transform_right)

        # # Compute the average position in the base frame
        # base_combined_x = (base_point_left.point.x + base_point_right.point.x) / 2
        # base_combined_y = (base_point_left.point.y + base_point_right.point.y) / 2
        # # base_combined_z = (base_point_left.point.z + base_point_right.point.z) / 2

        # # Get the combined GPS position in UTM coordinates
        # combined_base_utm_x = base_combined_x + ref_utm_x
        # combined_base_utm_y = base_combined_y + ref_utm_y

        # # Convert back to WGS84 coordinates
        # combined_base_lat, combined_base_lon = get_wgs84_coordinates(
        #     combined_base_utm_x, combined_base_utm_y)

        # # Publish the Combined GPS transform
        # if self.publish_combined_gps_tf:
        #     base_to_combined_gps_tf = TransformStamped()
        #     base_to_combined_gps_tf.header.stamp = self.get_clock().now().to_msg()
        #     base_to_combined_gps_tf.header.frame_id = self.base_frame_id
        #     base_to_combined_gps_tf.child_frame_id = self.combined_gps_frame_id
        #     base_to_combined_gps_tf.transform.translation.x = 0.0
        #     base_to_combined_gps_tf.transform.translation.y = 0.0
        #     base_to_combined_gps_tf.transform.translation.z = 0.0
        #     base_to_combined_gps_tf.transform.rotation.x = 0.0
        #     base_to_combined_gps_tf.transform.rotation.y = 0.0
        #     base_to_combined_gps_tf.transform.rotation.z = 0.0
        #     base_to_combined_gps_tf.transform.rotation.w = 1.0
        #     self.tf_broadcaster.sendTransform(base_to_combined_gps_tf)

        # # Publish the Combined GPS Fix
        # combined_gps_fix = NavSatFix()
        # combined_gps_fix.header.stamp = self.get_clock().now().to_msg()
        # combined_gps_fix.header.frame_id = self.combined_gps_frame_id
        # combined_gps_fix.latitude = combined_base_lat
        # combined_gps_fix.longitude = combined_base_lon
        # combined_gps_fix.altitude = combined_gps_altitude
        # self.gps_fix_pub.publish(combined_gps_fix)
        
        #TODO: fix combined_gps_fix
        combined_gps_fix = self.last_gps_right
        combined_gps_fix.header.stamp = self.get_clock().now().to_msg()
        self.combined_gps_frame_id = self.last_gps_right.header.frame_id
        self.gps_fix_pub.publish(combined_gps_fix)
        
        self.get_logger().info(f'Heading Yaw Angle: {heading_yaw}')
        heading_quaternion = R.from_euler('z', heading_yaw).as_quat()
        # self.get_logger().info(f'Heading Quaternion: {heading_quaternion}')

        # Publish the Dual GPS Heading as a PoseWithCovarianceStamped message
        # dual_gps_heading = PoseWithCovarianceStamped()
        # dual_gps_heading.header.stamp = self.get_clock().now().to_msg()
        # dual_gps_heading.header.frame_id = self.combined_gps_frame_id

        # # Set position (optional, set to zero)
        # dual_gps_heading.pose.pose.position.x = 0.0
        # dual_gps_heading.pose.pose.position.y = 0.0
        # dual_gps_heading.pose.pose.position.z = 0.0

        # # Convert heading to quaternion
        # dual_gps_heading.pose.pose.orientation.x = heading_quaternion[0]
        # dual_gps_heading.pose.pose.orientation.y = heading_quaternion[1]
        # dual_gps_heading.pose.pose.orientation.z = heading_quaternion[2]
        # dual_gps_heading.pose.pose.orientation.w = heading_quaternion[3]

        # # Set covariance
        # covariance = [0.0] * 36
        # # Set diagonal entries (variances) to small non-zero values
        # covariance[0] = 1e-3  # Variance for x
        # covariance[7] = 1e-3  # Variance for y
        # covariance[14] = 1e-3  # Variance for z
        # covariance[21] = 1e-3  # Variance for roll
        # covariance[28] = 1e-3  # Variance for pitch
        # covariance[35] = 0.01  # Variance for yaw (small uncertainty)
        # dual_gps_heading.pose.covariance = [
        #     0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0479, 0.0, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0207, 0.0,
        #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0041
        # ]
        # self.last_heading_pose = dual_gps_heading
        # self.dual_gps_heading_pub.publish(dual_gps_heading)
        
        imu_gps_heading = Imu()
        imu_gps_heading.header.stamp = self.get_clock().now().to_msg()
        # imu_gps_heading.header.frame_id = self.combined_gps_frame_id #"imu_link"
        imu_gps_heading.header.frame_id = "imu_link"
        imu_gps_heading.orientation.x = heading_quaternion[0]
        imu_gps_heading.orientation.y = heading_quaternion[1]
        imu_gps_heading.orientation.z = heading_quaternion[2]
        imu_gps_heading.orientation.w = heading_quaternion[3]
        
        imu_gps_heading.orientation_covariance = [
            0.0479, 0.0, 0.0,
            0.0, 0.020, 0.0,
            0.0, 0.0, 0.0041
        ]
        
        self.imu_gps_heading_pub.publish(imu_gps_heading)
        # Reset the update flags
        self.gps_left_updated = False
        self.gps_right_updated = False

def main(args=None):
    rclpy.init(args=args)
    dual_gps_compass = DualGpsCompass()
    rclpy.spin(dual_gps_compass)
    dual_gps_compass.destroy_node()
    rclpy.shutdown()
