import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from rtcm_msgs.msg import Message as Rtcm
import tkinter as tk
from tkinter import ttk


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
        }

        self.gui_update_callback = None  # Initialize as None

        # Subscriptions for the topics
        self.create_subscription(Imu, '/imu/data', self.generic_callback('/imu/data'), 10)
        self.create_subscription(LaserScan, '/scan', self.generic_callback('/scan'), 10)
        self.create_subscription(Rtcm, '/rtcm', self.generic_callback('/rtcm'), 10)
        self.create_subscription(Imu, '/imu_gps_heading/data', self.generic_callback('/imu_gps_heading/data'), 10)
        self.create_subscription(NavSatFix, '/ublox_gpsl_node/fix', self.generic_callback('/ublox_gpsl_node/fix'), 10)
        self.create_subscription(NavSatFix, '/ublox_gpsr_node/fix', self.generic_callback('/ublox_gpsr_node/fix'), 10)

    def generic_callback(self, topic):
        def callback(msg):
            self.last_received_times[topic] = self.get_clock().now()
        return callback

    def check_sensor_status(self):
        now = self.get_clock().now()
        status = {}

        for topic, last_time in self.last_received_times.items():
            if last_time is None:
                status[topic] = ("No data yet", "black")
            else:
                elapsed_time = (now - last_time).nanoseconds / 1e9
                if elapsed_time < 2.0:
                    status[topic] = ("Active", "green")
                else:
                    status[topic] = ("Inactive", "red")

        # Call the GUI update callback if it's set
        if self.gui_update_callback:
            self.gui_update_callback(status)


class SensorMonitorGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node

        self.root.title("AMR Health Monitor")
        self.root.geometry("300x200")
        self.root.configure(bg="#f0f0f0")

        # Treeview for the table
        self.tree = ttk.Treeview(
            self.root, columns=("Name", "State"), show="", height=10
        )
        self.tree.column("Name", width=150, anchor="w")
        self.tree.column("State", width=100, anchor="center")
        self.tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Add alternating row colors
        style = ttk.Style()
        style.configure("Treeview", rowheight=25, font=("Helvetica", 12))
        style.map(
            "Treeview",
            background=[("selected", "#4CAF50")],
            foreground=[("selected", "white")],
        )
        self.tree.tag_configure("even", background="#f9f9f9")
        self.tree.tag_configure("odd", background="#e0e0e0")
        self.tree.tag_configure("active", foreground="green")
        self.tree.tag_configure("inactive", foreground="red")
        self.tree.tag_configure("waiting", foreground="black")

        # Mapping of ROS topics to display names
        self.display_names = {
            '/imu/data': 'IMU',
            '/scan': '2D Lidar',
            '/rtcm': 'RTCM',
            '/ublox_gpsl_node/fix': 'Left GPS',
            '/ublox_gpsr_node/fix': 'Right GPS',
            '/imu_gps_heading/data': 'Heading',
        }

        # Initialize the table with placeholders
        self.table_data = {
            topic: ("Waiting...", "waiting") for topic in self.display_names
        }
        self.update_table()

        # Start updating the GUI
        self.update_gui()

    def update_status(self, status):
        for topic, (state, color) in status.items():
            tag = "active" if color == "green" else "inactive" if color == "red" else "waiting"
            self.table_data[topic] = (state, tag)
        self.update_table()

    def update_table(self):
        # Clear existing rows
        for row in self.tree.get_children():
            self.tree.delete(row)

        # Insert updated rows with alternating colors and state-based text colors
        for i, (topic, (state, tag)) in enumerate(self.table_data.items()):
            display_name = self.display_names.get(topic, topic)
            row_tag = "even" if i % 2 == 0 else "odd"
            self.tree.insert("", "end", values=(display_name, state), tags=(row_tag, tag))

    def update_gui(self):
        # Check sensor status and update the GUI
        self.ros_node.check_sensor_status()

        # Spin ROS2 to process incoming messages
        rclpy.spin_once(self.ros_node, timeout_sec=0.1)

        # Schedule the next update
        self.root.after(100, self.update_gui)


def main(args=None):
    rclpy.init(args=args)

    # Create the ROS2 node
    sensor_monitor_node = SensorMonitorNode()

    # Create the Tkinter GUI
    root = tk.Tk()
    gui = SensorMonitorGUI(root, sensor_monitor_node)

    # Set the GUI update callback in the ROS2 node
    sensor_monitor_node.gui_update_callback = gui.update_status

    try:
        # Run the Tkinter mainloop
        root.mainloop()
    finally:
        sensor_monitor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
