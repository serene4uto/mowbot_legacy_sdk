import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import tkinter as tk
from tkinter import filedialog, messagebox

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.map_data = None

    def map_callback(self, msg):
        self.get_logger().info("Map data received!")
        self.map_data = msg

    def save_map(self, file_path):
        if not self.map_data:
            messagebox.showerror("Error", "No map data received yet!")
            self.get_logger().error("No map data received to save.")
            return

        # Save the map image
        map_image_path = file_path + '.pgm'
        with open(map_image_path, 'w') as image_file:
            width, height = self.map_data.info.width, self.map_data.info.height
            image_file.write(f"P2\n{width} {height}\n255\n")
            for y in range(height):
                for x in range(width):
                    index = x + y * width
                    pixel_value = 255 if self.map_data.data[index] == -1 else \
                                  (0 if self.map_data.data[index] <= 25 else 100)
                    image_file.write(f"{pixel_value} ")
                image_file.write("\n")

        # Save the map YAML file with fixed format
        map_yaml_path = file_path + '.yaml'
        map_metadata = {
            'image': map_image_path.split('/')[-1],
            'mode': 'trinary',
            'resolution': round(self.map_data.info.resolution, 2),
            'origin': [
                round(self.map_data.info.origin.position.x, 2),
                round(self.map_data.info.origin.position.y, 2),
                0.0  # Assuming no rotation
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        # Write YAML manually to enforce exact formatting
        with open(map_yaml_path, 'w') as yaml_file:
            yaml_file.write(f"image: {map_metadata['image']}\n")
            yaml_file.write(f"mode: {map_metadata['mode']}\n")
            yaml_file.write(f"resolution: {map_metadata['resolution']}\n")
            yaml_file.write(f"origin: {map_metadata['origin']}\n")
            yaml_file.write(f"negate: {map_metadata['negate']}\n")
            yaml_file.write(f"occupied_thresh: {map_metadata['occupied_thresh']}\n")
            yaml_file.write(f"free_thresh: {map_metadata['free_thresh']}\n")

        messagebox.showinfo("Success", f"Map saved to {map_yaml_path} and {map_image_path}")
        self.get_logger().info(f"Map saved to {map_yaml_path} and {map_image_path}")


def save_map_gui(map_saver):
    file_path = filedialog.asksaveasfilename(
        defaultextension=".yaml",
        filetypes=[("YAML files", "*.yaml")],
        title="Save Map"
    )
    if file_path:
        map_saver.save_map(file_path.rsplit('.', 1)[0])


def main():
    rclpy.init()
    map_saver = MapSaver()

    def spin_once():
        rclpy.spin_once(map_saver, timeout_sec=0.1)
        root.after(100, spin_once)

    # Tkinter GUI
    root = tk.Tk()
    root.title("Map Saver App")

    label = tk.Label(root, text="ROS2 Map Saver", font=("Arial", 16))
    label.pack(pady=10)

    save_button = tk.Button(
        root,
        text="Save Map",
        font=("Arial", 14),
        command=lambda: save_map_gui(map_saver)
    )
    save_button.pack(pady=20)

    exit_button = tk.Button(
        root,
        text="Exit",
        font=("Arial", 14),
        command=lambda: [map_saver.destroy_node(), rclpy.shutdown(), root.destroy()]
    )
    exit_button.pack(pady=20)

    root.after(100, spin_once)
    root.mainloop()


if __name__ == '__main__':
    main()
