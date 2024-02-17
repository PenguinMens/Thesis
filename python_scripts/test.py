import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'pico_odometry',  # Replace 'odometry_topic' with the actual topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x_velocity = msg.twist.twist.linear.x
        angular_x_velocity = msg.twist.twist.angular.x
        angular_y_velocity = msg.twist.twist.angular.y
        print(f'\rX Speed: {x_velocity:.2f} m/s, Angular X: {angular_x_velocity:.2f} rad/s, Angular Y: {angular_y_velocity:.2f} rad/s', end='', flush=True)

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()
    try:
        rclpy.spin(odometry_subscriber)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: shutting down...")
    except Exception as e:
        print(f"\nFailed to connect to topic: {e}")
    finally:
        odometry_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
