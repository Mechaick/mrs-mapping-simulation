import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry

class TFSubscriber(Node):

    def __init__(self):
        super().__init__('relative_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('robot_name', '/tb3_0/')
        self.robot_namespace =  self.get_parameter('robot_name').get_parameter_value().string_value + '/'
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info("RELATIVE TF ___________________________")
        self.get_logger().info(f'subscribing to = {self.robot_namespace}odom')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, 'odom_relative', 10)
        self.timer = self.create_timer(0.1, self.timer_callback),
        self.initialized = False
        self.init_odom = Odometry()
        self.current_odom = Odometry()

    def odom_callback(self, msg):
        if not self.initialized:
            self.init_odom = msg
            self.initialized = True
            self.get_logger().info(f'init_odom = {self.init_odom}')

        else:

            relative_odometry = Odometry()
            relative_odometry.header.frame_id = 'world'
            relative_odometry.child_frame_id = self.robot_namespace+'odom'
            relative_odometry.header.stamp = self.get_clock().now().to_msg()
            relative_odometry.pose.pose.position.x = msg.pose.pose.position.x - self.init_odom.pose.pose.position.x
            relative_odometry.pose.pose.position.y = msg.pose.pose.position.y - self.init_odom.pose.pose.position.y
            relative_odometry.pose.pose.position.z = msg.pose.pose.position.z - self.init_odom.pose.pose.position.z
            relative_odometry.pose.pose.orientation.x = msg.pose.pose.orientation.x
            relative_odometry.pose.pose.orientation.y = msg.pose.pose.orientation.y
            relative_odometry.pose.pose.orientation.z = msg.pose.pose.orientation.z
            relative_odometry.pose.pose.orientation.w = msg.pose.pose.orientation.w
            self.pub.publish(relative_odometry)
            self.current_odom = relative_odometry
            #self.get_logger().info(f'tf = {world_map_transform}')

    def timer_callback(self):
        # print('timer_callback')
        # print(rclpy.time.Time())
        # print(self.get_clock().now())
        # odom_transform = self.tf_buffer.lookup_transform(f'{self.robot_namespace}odom_real', f'world',rclpy.time.Time(),
        # timeout=rclpy.duration.Duration(seconds=1.0))
        # base_transform = self.tf_buffer.lookup_transform(f'{self.robot_namespace}map', f'{self.robot_namespace}base_footprint', rclpy.time.Time())

        world_map_transform = geometry_msgs.msg.TransformStamped()
        world_map_transform.header.stamp = self.get_clock().now().to_msg()
        world_map_transform.header.frame_id = f'world'
        world_map_transform.child_frame_id = f'{self.robot_namespace}odom'
        #self.get_logger().info(f'Transfom to =  {world_map_transform.child_frame_id}')
        # Calculate the transformation between world and map
        world_map_transform.transform.translation.x = self.init_odom.pose.pose.position.x
        world_map_transform.transform.translation.y = self.init_odom.pose.pose.position.y
        world_map_transform.transform.translation.z = self.init_odom.pose.pose.position.z
        world_map_transform.transform.rotation.x = self.init_odom.pose.pose.orientation.x
        world_map_transform.transform.rotation.y = self.init_odom.pose.pose.orientation.y
        world_map_transform.transform.rotation.z = self.init_odom.pose.pose.orientation.z
        world_map_transform.transform.rotation.w = self.init_odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(world_map_transform)
        world_map_transform.header.stamp = self.get_clock().now().to_msg()
        world_map_transform.header.frame_id = f'{self.robot_namespace}odom'
        world_map_transform.child_frame_id = f'{self.robot_namespace}base_footprint'
        world_map_transform.transform.translation.x = self.current_odom.pose.pose.position.x 
        world_map_transform.transform.translation.y = self.current_odom.pose.pose.position.y
        world_map_transform.transform.translation.z = self.current_odom.pose.pose.position.z
        world_map_transform.transform.rotation.x = self.current_odom.pose.pose.orientation.x
        world_map_transform.transform.rotation.y = self.current_odom.pose.pose.orientation.y
        world_map_transform.transform.rotation.z = self.current_odom.pose.pose.orientation.z
        world_map_transform.transform.rotation.w = self.current_odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(world_map_transform)
        #self.get_logger().info(f'tf = {world_map_transform}')
        #self.pub.publish(world_map_transform)


def main():
    rclpy.init()
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    tf_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
