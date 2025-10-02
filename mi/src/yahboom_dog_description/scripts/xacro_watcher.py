#!/usr/bin/env python3
import os
import subprocess
import rclpy
from rclpy.node import Node
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class XacroWatcher(Node):
    def __init__(self, xacro_file):
        super().__init__('xacro_watcher')
        self.xacro_file = xacro_file
        self.declare_parameter('robot_description', '')

        # Lần đầu load
        self.reload_urdf()

        # Watchdog setup
        event_handler = FileSystemEventHandler()
        event_handler.on_modified = self.on_modified
        self.observer = Observer()
        self.observer.schedule(event_handler, os.path.dirname(self.xacro_file), recursive=False)
        self.observer.start()

        self.get_logger().info(f'Watching {self.xacro_file} for changes...')

    def reload_urdf(self):
        try:
            urdf = subprocess.check_output(['xacro', self.xacro_file]).decode()
            self.set_parameters([rclpy.parameter.Parameter(
                'robot_description', rclpy.Parameter.Type.STRING, urdf)])
            self.get_logger().info('Reloaded URDF successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to reload URDF: {e}')

    def on_modified(self, event):
        if event.src_path == self.xacro_file:
            self.get_logger().info('Xacro file changed, reloading...')
            self.reload_urdf()

    def destroy_node(self):
        self.observer.stop()
        self.observer.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    xacro_file = os.path.expanduser('~/mi/src/yahboom_dog_description/urdf/yahboom_dog.urdf.xacro')
    node = XacroWatcher(xacro_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

