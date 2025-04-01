import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import tkinter as tk
from tkinter import filedialog
from geometry_msgs.msg import PoseStamped
import math
from scipy.spatial.transform import Rotation as R


class YamlWaypointParser:
    """
    Parse a set of waypoints from a YAML file.
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        wps = []
        for wp in self.wps_dict["waypoints"]:
            x = wp["x"]
            y = wp["y"]
            yaw = wp["yaw"]
            wps.append((x, y, yaw))
        return wps


class WaypointFollowerGUI(tk.Tk, Node):

    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'waypoint_follower')

        self.title("Waypoint Follower GUI")
        self.resizable(False, False)

        # GUI elements
        self.load_label = tk.Label(self, text="File path:")
        self.load_textbox = tk.Entry(self, width=45)
        self.load_textbox.insert(0, "waypoints.yaml")
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

        # ROS2 components
        self.navigator = BasicNavigator()
        self.wp_parser = None

        self.state_check_timer = self.create_timer(1, self.check_nav_state)
        self.state_check_timer.cancel()

    def select_path(self):
        self.load_textbox.delete(0, tk.END)
        self.load_textbox.insert(0, filedialog.askopenfilename())

    def start_wpf(self):
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        # Parse waypoints from YAML file
        self.wp_parser = YamlWaypointParser(self.load_textbox.get())
        wps = self.wp_parser.get_wps()

        wpl = []
        for x, y, yaw in wps:
            # Create PoseStamped message
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Convert yaw to quaternion
            yaw_rad = yaw
            rotation = R.from_euler('z', yaw_rad)
            quaternion = rotation.as_quat()  # Returns [x, y, z, w]

            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            wpl.append(pose)

        # Command the robot to follow the waypoints
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
            self.get_logger().info("Waypoints completed successfully")
            self.stop_wpf()


def main(args=None):
    rclpy.init(args=args)

    waypoint_flw = WaypointFollowerGUI()

    while rclpy.ok():
        # Spin ROS and update the Tkinter GUI
        rclpy.spin_once(waypoint_flw, timeout_sec=0.1)
        waypoint_flw.update()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
