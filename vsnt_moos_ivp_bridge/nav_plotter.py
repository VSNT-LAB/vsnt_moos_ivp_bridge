import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose2D

class MoosSubscriber(Node):
    """
    Subscriber node for messages that came from MOOS
    """
    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            Pose2D,
            'vsnt/position',
            self.listener_callback,
            10)
        self.subscription
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.line, = self.ax.plot([], [], color='yellow')
        self.x_coords = []
        self.y_coords = []


    def listener_callback(self,position_msg: Pose2D):
        """
        Callback for the position subscriber
        """
        x = position_msg.x
        y = position_msg.y
        theta = position_msg.theta
        self.get_logger().info(f'Get NAV Position: X={x},Y={y},Heading={theta}')
        self.x_coords.append(x)
        self.y_coords.append(y)
            
        self.ax.plot(self.x_coords, self.y_coords, color='yellow')
        self.ax.set_xlim(-4000,-1000)
        self.ax.set_ylim(1000,4000)
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    moos_subscriber = MoosSubscriber()
    rclpy.spin(moos_subscriber)

if __name__ == '__main__':
    main()