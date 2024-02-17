import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel = Twist()

    def publish_cmd_vel(self):
        self.publisher_.publish(self.cmd_vel)
        self.get_logger().info('Publishing cmd_vel: Linear x: %.2f, Angular z: %.2f' %
                               (self.cmd_vel.linear.x, self.cmd_vel.angular.z))

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    try:
        while rclpy.ok():
            linear_x = float(input("Enter linear x velocity: "))
            angular_z = float(input("Enter angular z velocity: "))

            cmd_vel_publisher.cmd_vel.linear.x = linear_x
            cmd_vel_publisher.cmd_vel.angular.z = angular_z

            cmd_vel_publisher.publish_cmd_vel()

    except ValueError:
        print("Invalid input! Please enter numeric values.")
    except Exception as e:
        print(f"\nFailed to publish cmd_vel: {e}")
    finally:
        cmd_vel_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
