import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from py_mowbot_utils.gps_utils import latLonYaw2Geopose
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from sensor_msgs.msg import NavSatFix
import os
import sys
import yaml

from sensor_msgs.msg import Imu

from scipy.spatial.transform import Rotation as R
import math


class GpsWaypointPlannerGUI(tk.Tk, Node):
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoints_planner')
        self.title("GPS Waypoints Planner GUI")
        
        self.resizable(False, False)

        self.instructions1_label = tk.Label(self, 
                text="!!! Click on the mapviz to select the waypoints",
                    justify=tk.CENTER)
        self.instructions2_label = tk.Label(self,
                text="!!! Double click on the table to edit the waypoints",
                    justify=tk.CENTER)

        self.gps_waypoints_label = tk.Label(self, text="Selected waypoints:")

        self.treeview = ttk.Treeview(self)
        self.treeview["columns"]=("Order","Latitude","Longitude","Yaw")

        # Format our columns
        self.treeview.column("#0", width=0, stretch=tk.NO)
        self.treeview.column("Order", anchor=tk.W, width=50)
        self.treeview.column("Latitude", anchor=tk.W, width=120)
        self.treeview.column("Longitude", anchor=tk.W, width=120)
        self.treeview.column("Yaw", anchor=tk.W, width=120)

        # Create Headings
        self.treeview.heading("#0", text="", anchor=tk.W)
        self.treeview.heading("Order", text="Order", anchor=tk.W)
        self.treeview.heading("Latitude", text="Latitude", anchor=tk.W)
        self.treeview.heading("Longitude", text="Longitude", anchor=tk.W)
        self.treeview.heading("Yaw", text="Yaw (rad)", anchor=tk.W)
        self.treeview.bind("<Double-1>", self.on_double_click)

        self.save_name_label = tk.Label(self, text="Map Name:")
        self.save_name_textbox = tk.Entry(self, width=45)
        self.save_name_textbox.insert(0, "gps_waypoints.yaml")

        self.save_button = tk.Button(self, text="Save",
                                           command=self.save_selected_waypoints)
        self.clear_button = tk.Button(self, text="Clear",
                                           command=self.clear_selected_waypoints)
        
        
        self.instructions1_label.grid(row=0, column=0, columnspan=3, sticky='ew')
        self.instructions2_label.grid(row=1, column=0, columnspan=3, sticky='ew')
        self.gps_waypoints_label.grid(row=2, column=0, columnspan=3, sticky='w')
        self.treeview.grid(row=3, column=0, columnspan=3, sticky='nsew')
        self.save_name_label.grid(row=4, column=0)
        self.save_name_textbox.grid(row=4, column=1)
        self.save_button.grid(row=5, column=0, sticky='ew')
        self.clear_button.grid(row=5, column=2, sticky='ew')


        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)
        
        self.selected_wps_pub = self.create_publisher(
            NavSatFix, "/igw_gps_points", 1)
        
        self.orientation_sub = self.create_subscription(
            Imu, "/imu_gps_heading/data", self.orientation_cb, 1)

        self.last_gps_position = NavSatFix()
        self.last_heading = 0.0

        self.waypoints_counter = 0

        # Store waypoints in a list
        self.waypoints = []

    def orientation_cb(self, msg: Imu):
        # convert quaternion to euler
        r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        yaw = r.as_euler('zyx')[0]
        self.last_heading = yaw
        self.get_logger().info(f"Received orientation from imu_gps_heading at yaw: {yaw}")

    def mapviz_wp_cb(self, msg: PointStamped):
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that is not in wgs84 frame. This is not a gps point and won't be followed")
            return
        
        self.get_logger().info(f"Received point from mapviz at lon: {msg.point.x}, lat: {msg.point.y}")
        
        self.last_gps_position.latitude = msg.point.y
        self.last_gps_position.longitude = msg.point.x
        self.selected_wps_pub.publish(self.last_gps_position)

        self.waypoints_counter += 1

        # Add the waypoint to the list
        waypoint = {
            'order': self.waypoints_counter,
            'latitude': msg.point.y,
            'longitude': msg.point.x,
            'yaw': 0.0  # Placeholder for yaw
        }
        self.waypoints.append(waypoint)

        # Update yaw of previous waypoint if it exists
        if len(self.waypoints) > 1:
            # Calculate yaw from previous point to this point
            prev_wp = self.waypoints[-2]
            curr_wp = self.waypoints[-1]
            yaw = self.calculate_yaw(prev_wp['latitude'], prev_wp['longitude'],
                                     curr_wp['latitude'], curr_wp['longitude'])
            # Update previous waypoint's yaw
            self.waypoints[-2]['yaw'] = yaw
            # Set current waypoint's yaw to be the same as previous waypoint's yaw
            self.waypoints[-1]['yaw'] = yaw
        else:
            # First waypoint, set yaw to zero
            self.waypoints[-1]['yaw'] = 0.0

        # Update the Treeview
        self.update_treeview()
        
    def on_double_click(self, event):
        # Get the selected item
        item = self.treeview.selection()[0]

        # Determine the column clicked
        column = self.treeview.identify_column(event.x)
        # col_index = self.treeview["columns"].index(column)-1
        if column == '#1':
            col_index = 0
        elif column == '#2':
            col_index = 1
        elif column == '#3':
            col_index = 2
        elif column == '#4':
            col_index = 3
        else:
            return  # Clicked outside the defined columns

        # Create an entry widget to edit the cell value
        x, y, width, height = self.treeview.bbox(item, column)
        entry = tk.Entry(self, width=width)
        entry.place(x=x, y=y, width=width, height=height)

        def on_entry_confirm(event):
            # Update the treeview item
            self.treeview.set(item, column=column, value=entry.get())
            entry.destroy()
            # Update the waypoint data
            index = int(self.treeview.index(item))
            value = entry.get()
            if col_index == 1:
                self.waypoints[index]['latitude'] = float(value)
            elif col_index == 2:
                self.waypoints[index]['longitude'] = float(value)
            elif col_index == 3:
                self.waypoints[index]['yaw'] = float(value)
            # Recalculate yaw if latitude or longitude changed
            if col_index == 1 or col_index == 2:
                self.update_yaw_after_edit(index)
                self.update_treeview()

        def on_entry_click_outside(event):
            entry.destroy()

        # Bind events to handle editing completion
        entry.bind("<Return>", on_entry_confirm)
        entry.bind("<FocusOut>", on_entry_click_outside)

        # Set the entry with the current cell value and focus it
        entry.insert(0, self.treeview.item(item, 'values')[col_index])
        entry.select_range(0, tk.END)
        entry.focus_set()

    def save_selected_waypoints(self):

        waypoint_data_list = {"waypoints": []}
        logging_file_path = self.save_name_textbox.get()
            
        with open(logging_file_path, "w") as f:
            for wp in self.waypoints:
                data = {
                    "latitude": wp['latitude'],
                    "longitude": wp['longitude'],
                    "yaw": wp['yaw'],
                }
                waypoint_data_list["waypoints"].append(data)
            yaml.dump(waypoint_data_list, f)
        messagebox.showinfo("Saved", f"Saved waypoints to {logging_file_path}")

    def clear_selected_waypoints(self):
        self.waypoints_counter = 0
        self.treeview.delete(*self.treeview.get_children())
        self.waypoints = []

    def calculate_yaw(self, lat1_deg, lon1_deg, lat2_deg, lon2_deg):
        """
        Calculates the yaw between two points in radians, from -pi to pi,
        with 0 pointing East and pi/2 pointing North.
    
        Parameters:
        lat1_deg, lon1_deg: Latitude and Longitude of point 1 (in degrees)
        lat2_deg, lon2_deg: Latitude and Longitude of point 2 (in degrees)
    
        Returns:
        Yaw in radians from -pi to pi, with 0 pointing East.
        """
        # Convert latitudes and longitudes from degrees to radians
        lat1 = math.radians(lat1_deg)
        lon1 = math.radians(lon1_deg)
        lat2 = math.radians(lat2_deg)
        lon2 = math.radians(lon2_deg)
    
        # Calculate differences
        d_lon = lon2 - lon1
    
        # Calculate components
        x = math.sin(d_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - \
            math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    
        # Calculate yaw angle from East, counterclockwise positive
        yaw = math.atan2(y, x)
    
        # Normalize yaw to -pi to pi
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    
        return yaw



    def update_treeview(self):
        # Clear the Treeview
        self.treeview.delete(*self.treeview.get_children())
        # Re-insert all waypoints with updated yaw
        for wp in self.waypoints:
            self.treeview.insert(parent='', index='end', text='',
                                 values=(wp['order'], wp['latitude'], wp['longitude'], wp['yaw']))

    def update_yaw_after_edit(self, index):
        # Update yaw of the waypoint at the given index and its neighbors if necessary
        wp = self.waypoints[index]
        # Update previous waypoint's yaw if current waypoint is not the first one
        if index > 0:
            prev_wp = self.waypoints[index - 1]
            yaw = self.calculate_yaw(prev_wp['latitude'], prev_wp['longitude'],
                                     wp['latitude'], wp['longitude'])
            prev_wp['yaw'] = yaw
            wp['yaw'] = yaw  # Set current waypoint's yaw to be the same as previous
        else:
            # First waypoint, set its yaw to zero or calculate if there's a next waypoint
            if len(self.waypoints) > 1:
                next_wp = self.waypoints[index + 1]
                yaw = self.calculate_yaw(wp['latitude'], wp['longitude'],
                                         next_wp['latitude'], next_wp['longitude'])
                wp['yaw'] = yaw
            else:
                wp['yaw'] = 0.0
        # Update next waypoint's yaw if exists
        if index + 1 < len(self.waypoints):
            next_wp = self.waypoints[index + 1]
            yaw = self.calculate_yaw(wp['latitude'], wp['longitude'],
                                     next_wp['latitude'], next_wp['longitude'])
            wp['yaw'] = yaw
            next_wp['yaw'] = yaw  # Keep next waypoint's yaw same as current

def main(args=None):
    rclpy.init(args=args)

    igps_gui_logger = GpsWaypointPlannerGUI()

    while rclpy.ok():
        # we spin both the ROS system and the interface
        rclpy.spin_once(igps_gui_logger, timeout_sec=0.1)  # Run ros2 callbacks
        igps_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()
