import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import time
import tkinter as tk
from tkinter import filedialog
from py_mowbot_utils.gps_utils import latLonYaw2Geopose
from sensor_msgs.msg import NavSatFix

from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWaypointFollowerGUI(tk.Tk, Node):
        
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoint_follower')

        self.title("GPS Waypoint Follower GUI")
        self.resizable(False, False)

        self.load_label = tk.Label(self, text="File path:")
        self.load_textbox = tk.Entry(self, width=45)
        self.load_textbox.insert(0, "gps_waypoints.yaml")
        self.select_path_button = tk.Button(self, text="Select",
                                            command=self.select_path)
        
        self.start_button = tk.Button(self, text="Start",
                                            command=self.start_wpf)
        
        self.stop_button = tk.Button(self, text="Stop",
                                            command=self.stop_wpf)
        self.stop_button.config(state=tk.DISABLED)
        
        self.load_label.grid(row=0, column=0, sticky='ew')
        self.load_textbox.grid(row=0, column=1, sticky='ew')
        self.select_path_button.grid(row=0, column=2, sticky='ew')
        self.start_button.grid(row=1, column=0, sticky='ew')
        self.stop_button.grid(row=1, column=2, sticky='ew')

        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = None
        self.localizer = self.create_client(FromLL,  '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.state_check_timer = self.create_timer(1, self.check_nav_state)
        self.state_check_timer.cancel()

        self.selected_wps_pub = self.create_publisher(
            NavSatFix, "/igw_gps_points", 1)
        
        self.prob_wps_pose_pub = self.create_publisher(
            PoseStamped, "/igw_prob_wp", 1)
        
        

    def select_path(self):
        self.load_textbox.delete(0, tk.END)
        self.load_textbox.insert(0, filedialog.askopenfilename())
    
    def start_wpf(self):

        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.wp_parser = YamlWaypointParser(self.load_textbox.get())

        
        wps = self.wp_parser.get_wps()

        for wp in wps:
            loaded_wp_msg = NavSatFix()
            loaded_wp_msg.latitude = wp.position.latitude
            loaded_wp_msg.longitude = wp.position.longitude
            self.selected_wps_pub.publish(loaded_wp_msg)
            time.sleep(0.5)
        

        self.navigator.waitUntilNav2Active(localizer='controller_server')
        wpl = []
        for wp in wps:
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            log = 'long{:f}, lat={:f}, alt={:f}'.format(self.req.ll_point.longitude, self.req.ll_point.latitude, self.req.ll_point.altitude)
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
            self.prob_wps_pose_pub.publish(self.resp)
            time.sleep(0.5)

        self.get_logger().info(f'wpl={wpl}')

        self.navigator.followWaypoints(wpl)

        # Start the state check timer
        if self.state_check_timer.is_canceled():
            self.state_check_timer.reset()

    
    def stop_wpf(self):
        self.navigator.cancelTask()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

        # Stop the state check timer    
        if not self.state_check_timer.is_canceled():
            self.state_check_timer.cancel()

    def check_nav_state(self):
        if self.navigator.isTaskComplete():
            print("wps completed successfully")
            self.stop_wpf()


def main(args=None):
    rclpy.init(args=args)

    gps_waypoint_flw = GpsWaypointFollowerGUI()

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(gps_waypoint_flw, timeout_sec=0.1)  # Run ros2 callbacks
        gps_waypoint_flw.update()  # Update the tkinter interface

    rclpy.shutdown()