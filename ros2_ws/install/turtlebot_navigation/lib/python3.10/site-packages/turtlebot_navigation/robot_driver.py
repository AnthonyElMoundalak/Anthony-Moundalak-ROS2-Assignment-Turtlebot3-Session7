import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from my_interface.srv import FindClosestWall

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        
        # Create a service client to find the closest wall
        self.client = self.create_client(FindClosestWall, 'find_closest_wall')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = FindClosestWall.Request()

        # Initialize subscriber to the /scan topic and publisher to the /cmd_vel topic
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.twist = Twist()
        
        self.get_logger().info('Waiting for FindClosestWall service...')
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        if response.success:
            self.get_logger().info('Arrived to the closest wall...')
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        else:
            self.get_logger().error('Does not arrived to the wall...')

    

    def scan_callback(self, msg):
        if msg.ranges[0] < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -2.0
        elif msg.ranges[25] < 0.55:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.3
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)
    robot_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

