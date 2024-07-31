import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from my_interface.action import MeasureLapTime
import time

class LapTimeActionServer(Node):
    def __init__(self):
        super().__init__('lap_time_action_server')
        self._action_server = ActionServer(
            self,
            MeasureLapTime,
            'MeasureLapTime',
            self.execute_callback
        )

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.start_position_y = None
        self.start_position_x = None
        self.start_time = None
        self.lap_times = []
        self.start_lap = False
        self.lap_count = 0
        self.new_lap = False

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received lap time goal')
        
        while not self.start_lap:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        while self.start_position_x is None or self.start_position_y is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.start_position_x = self.current_position_x
            self.start_position_y = self.current_position_y
            self.start_time = self.current_time
            
        self.get_logger().info(f'x = {self.current_position_x}, y = {self.current_position_y}')
            

        feedback_msg = MeasureLapTime.Feedback()
        self.start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self)
            # self.get_logger().info(f'x = {self.current_position_x}, y = {self.current_position_y}')

            elapsed_time = time.time() - self.start_time
            feedback_msg.elapsed_time = float(elapsed_time)
            goal_handle.publish_feedback(feedback_msg)

            if self.current_position_y > 1.0 :
                self.new_lap = True

            if self.current_position_x > 1.0 and self.current_position_y > -0.01 and self.current_position_y < 0.01 and self.new_lap:
                self.new_lap = False
                lap_time = time.time() - self.start_time
                self.lap_times.append(lap_time)
                self.lap_count += 1
                self.get_logger().info(f'Lap {self.lap_count} completed in {lap_time:.2f} seconds')
                self.start_time = time.time()  # Reset start time for next lap

            # Check if the action was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Lap time goal canceled')
                return MeasureLapTime.Result()
            
            if self.lap_count > 3:
                break

        result = MeasureLapTime.Result()
        result.total_time = float(sum(self.lap_times))
        goal_handle.succeed()
        self.get_logger().info(f'Total time for all laps: {result.total_time:.2f} seconds')
        return result

    def scan_callback(self, msg):
        if min(msg.ranges) < 0.5:
            self.start_lap = True
        else:
            self.start_lap = False
            
    def odom_callback(self, msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    action_server = LapTimeActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

