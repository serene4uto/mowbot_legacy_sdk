import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from py_mowbot_utils.gps_utils import latLonYaw2Geopose
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        try:
            with open(wps_file_path, 'r') as wps_file:
                self.wps_dict = yaml.safe_load(wps_file)
        except FileNotFoundError:
            raise FileNotFoundError(f"File {wps_file_path} not found.")

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps
    

class WaypointFollower(Node):
    
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.declare_parameter('waypoints_file', 'gps_waypoints.yaml')
        
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        
        self.get_logger().info(f'waypoints_file={self.waypoints_file}')
        
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = None
        self.localizer = self.create_client(FromLL, '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.state_check_timer = self.create_timer(1, self.check_nav_state)
        self.state_check_timer.cancel()
        
        self.start_wpf()
        

    def start_wpf(self):

        self.wp_parser = YamlWaypointParser(
            self.waypoints_file
        )
        wps = self.wp_parser.get_wps()

        self.navigator.waitUntilNav2Active(localizer='controller_server')
        self.get_logger().info('Nav2 is active')
        
        wpl = []
        for wp in wps:
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            log = 'lon={:f}, lat={:f}, alt={:f}'.format(self.req.ll_point.longitude, self.req.ll_point.latitude, self.req.ll_point.altitude)
            self.get_logger().info(log)

            self.future = self.localizer.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            self.resp = PoseStamped()
            self.resp.header.frame_id = 'map'
            self.resp.header.stamp = self.get_clock().now().to_msg()
            self.resp.pose.position = self.future.result().map_point

            log = 'x={:f}, y={:f}, z={:f}'.format(self.future.result().map_point.x, self.future.result().map_point.y, self.future.result().map_point.z)
            self.get_logger().info(log)
            
            self.resp.pose.orientation = wp.orientation
            wpl += [self.resp]

        self.get_logger().info(f'wpl={wpl}')

        self.navigator.followWaypoints(wpl)

        # Start the state check timer
        if self.state_check_timer.is_canceled():
            self.state_check_timer.reset()

    def stop_wpf(self):
        self.navigator.cancelTask()

        # Stop the state check timer    
        if not self.state_check_timer.is_canceled():
            self.state_check_timer.cancel()

    def check_nav_state(self):
        if self.navigator.isTaskComplete():
            print("wps completed successfully")
            self.stop_wpf()


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()