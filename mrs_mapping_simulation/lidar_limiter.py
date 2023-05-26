import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
 
class lidar_limiter(Node):
 
    def __init__(self):
        super().__init__('lidar_limiter')
        self.declare_parameter('robot_name', 'tb3_0')
        self.robot_namespace =  self.get_parameter('robot_name').get_parameter_value().string_value + '/'
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan_unfiltered',
            self.listener_callback,
            10)
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        # prevent warning that self.my_subscriber is not used
        self.laser_sub
 
    def listener_callback(self, msg):
        cropped_laser = LaserScan()
        cropped_laser.header = msg.header
        cropped_laser.angle_min = msg.angle_min
        cropped_laser.angle_max = msg.angle_max
        cropped_laser.angle_increment = msg.angle_increment
        cropped_laser.time_increment = msg.time_increment
        cropped_laser.scan_time = msg.scan_time
        cropped_laser.range_min = msg.range_min
        cropped_laser.range_max = msg.range_max
        #self.get_logger().info(f'range = {msg.ranges[0]}')
        for i in msg.ranges:
            
            if math.isinf(i):
                cropped_laser.ranges.append(6.5)
            else:
                cropped_laser.ranges.append(i)
        #cropped_laser.ranges = msg.ranges[0:360]
        cropped_laser.intensities = msg.intensities[0:360]
        self.laser_pub.publish(cropped_laser)



        
 
 
def main(args=None):
    rclpy.init(args=args)
 
    my_simple_subscriber = lidar_limiter()
    rclpy.spin(my_simple_subscriber)
     # destroy the node when it is not used anymore
    my_simple_subscriber.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()