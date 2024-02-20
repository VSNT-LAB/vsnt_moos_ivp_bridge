import rclpy
import pymoos
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class Moos2ROS(Node):

    def __init__(self, moos_receiver):
        """
        ROS node to convert MOOS messages to ROS
        """
        super().__init__("moos_to_ros")
        #self.get_logger().info()
        self.moos_receiver = moos_receiver
        self.publisher = self.create_publisher(Pose2D, 'vsnt/position', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_position)

    def publish_position(self):
        """
        Publish Pose2D message position to vsnt/position topic
        """
        position_msg = Pose2D()
        position_msg.x = float(self.moos_receiver.nav_x)
        position_msg.y = float(self.moos_receiver.nav_y)
        position_msg.theta = float(self.moos_receiver.nav_heading)

        self.publisher.publish(position_msg)
        self.get_logger().info(f'Publishing NAV Position: X={position_msg.x},Y={position_msg.y},Heading={position_msg.theta}')

class MoosReceiver(pymoos.comms):
    def __init__(self, moos_community, moos_port):
        """
        Initiates MOOSComms, sets the callbacks and runs the loop
        """
        super(MoosReceiver, self).__init__()
        self.server = moos_community
        self.port = moos_port
        self.name = 'moosReceiver'
        self.iter = 0

        # Setup variables
        self.nav_x = 0
        self.nav_y = 0
        self.nav_yaw = 0 
        self.nav_speed = 0
        self.nav_depth = 0
        self.nav_heading = 0
        self.last_ais_msg = None
        self.view_seglist = None
        self.view_point = None
        self.deploy = None
        self.return_var = None
        self.bhv_settings = None # Current behavior
        self.ivphelm_bhv_active = None 

        self.set_on_connect_callback(self.__on_connect)
        self.set_on_mail_callback(self.__on_new_mail)
        status = self.run(self.server, self.port, self.name)

        self.init_time = pymoos.time()

    def __on_connect(self):
        """
        Register MOOS variables when connecting to server
        """
        # Vessel Variables
        self.register('NAV_X', 0)
        self.register('NAV_Y', 0)
        self.register('NAV_HEADING', 0)
        self.register('DESIRED_HEADING', 0)
        self.register('DESIRED_RUDDER', 0)
        self.register('NAV_SPEED', 0)
        self.register('DESIRED_SPEED', 0)
        self.register('NAV_DEPTH', 0)
        self.register('NAV_YAW', 0)
        self.register('DESIRED_THRUST', 0)

        # Autonomous Navigation Variables
        self.register('VIEW_SEGLIST', 0) 
        self.register('VIEW_POINT', 0) 
        self.register('BHV_SETTINGS', 0)
        self.register('IVPHELM_BHV_ACTIVE', 0) 

        return True

    def __on_new_mail(self):
        """
        Callback to register incoming messages
        """
        msg_list = self.fetch()

        for msg in msg_list:
            val = msg.double()

            if msg.name() == 'NAV_X':
                self.nav_x = val
            elif msg.name() == 'NAV_Y':
                self.nav_y = val
            elif msg.name() == 'NAV_HEADING':
                self.nav_heading = val
            elif msg.name() == 'DESIRED_HEADING':
                self.desired_heading = val
            elif msg.name() == 'SETPOINT_HEADING':
                self.setpoint_heading = val
            elif msg.name() == 'DESIRED_RUDDER':
                self.desired_rudder = val
            elif msg.name() == 'NAV_DEPTH':
                self.nav_depth = val
            elif msg.name() == 'NAV_SPEED':
                self.nav_speed = val
            elif msg.name() == 'DESIRED_SPEED':
                self.desired_speed = val
            elif msg.name() == 'DESIRED_THRUST':
                self.desired_thrust = val
            elif msg.name() == 'DESIRED_RUDDER':
                self.desired_rudder = val
            elif msg.name() == 'VIEW_SEGLIST':
                val = msg.string()
                self.view_seglist = val
            elif msg.name() == 'NAV_YAW': # Vessel's rudder
                self.nav_yaw = val
            elif msg.name() == 'VIEW_POINT': # Current autonomous target point
                val = msg.string()
                self.view_point = val
            elif msg.name() == 'BHV_SETTINGS': 
                val = msg.string()
                self.bhv_settings = val
            elif msg.name() == 'IVPHELM_BHV_ACTIVE':
                val = msg.string()
                self.ivphelm_bhv_active = val
                print(self.ivphelm_bhv_active)    

        return True

def main(args=None):
    moos_receiver = MoosReceiver("127.0.0.1", 9000)
    rclpy.init(args=None)
    ros_node = Moos2ROS(moos_receiver)
    rclpy.spin(ros_node)
    #node.destroy_node()
    #rclpy.shutdown()

if __name__ == "__main__":
    main()