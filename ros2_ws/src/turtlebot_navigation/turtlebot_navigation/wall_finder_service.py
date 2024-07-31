import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from my_interface.srv import FindClosestWall
import numpy as np

class WallFinderService(Node):
    def __init__(self):
        super().__init__('wall_finder_service')
        
        # Create a service server for FindClosestWall
        self.srv = self.create_service(FindClosestWall, 'find_closest_wall', self.find_closest_wall_callback)
        
        # Initialize publisher to the /cmd_vel topic and subscriber to the /scan topic
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.twist = Twist()
        self.closest_wall_distance = float('inf')
        self.get_logger().info('Starting service...')

    def find_closest_wall_callback(self, request, response):
        self.get_logger().info('Finding closest wall...')
        self.twist.linear.x = 0.2
        self.vel_pub.publish(self.twist)
        
        self.timer = self.create_timer(0.1, self.check_closest_distance)
        self.get_logger().info('Moving forward...')

        # Send success response
        response.success = True
        return response

    def check_closest_distance(self):
        if self.closest_wall_distance < 0.5:
            self.twist.linear.x = 0.0
            self.vel_pub.publish(self.twist)
            self.get_logger().info('Arrive to the wall...')
            self.timer.cancel()
        else:
            self.twist.linear.x = 0.2
            self.vel_pub.publish(self.twist)
            self.get_logger().info(f'Min distance: {self.closest_wall_distance:.2f}')

    def scan_callback(self, msg):
        # Find the minimum distance to a wall in the laser scan data
        self.closest_wall_distance = min(msg.ranges)
        
        
        

def main(args=None):
    rclpy.init(args=args)
    wall_finder_service = WallFinderService()
    rclpy.spin(wall_finder_service)
    wall_finder_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

