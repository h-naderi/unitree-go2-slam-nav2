import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from unitree_api.msg import Request

class Go2CmdProcessor(Node):
    def __init__(self):
        super().__init__("go2_cmd_processor")

        # Declare parameters
        self.declare_parameter("rate", 200.0)
        self.declare_parameter("cmd_vel_timeout", 0.25)  # Timeout in seconds
        self.declare_parameter("log_level", "info")  # Logging level

        # Set the logger level based on the parameter
        log_level = self.get_parameter("log_level").value.lower()
        log_severity = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
            "fatal": LoggingSeverity.FATAL,
        }.get(log_level, LoggingSeverity.INFO)
        self.get_logger().set_level(log_severity)

        self.rate = self.get_parameter("rate").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value

        # Time intervals
        self.publish_interval = 1.0 / self.rate
        self.cmd_vel_timeout_duration = self.cmd_vel_timeout
        self.cmd_vel_counter = 0.0

        # Publishers
        self.publisher = self.create_publisher(Request, "/api/sport/request", 10)

        # Subscribers
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Services
        self.create_service(Empty, "stand_up", self.stand_up_callback)
        self.create_service(Empty, "lay_down", self.lay_down_callback)
        self.create_service(Empty, "recover_stand", self.recover_stand_callback)
        self.create_service(Empty, "damping", self.damping_callback)
        self.create_service(Empty, "dance1", self.dance1_callback)
        self.create_service(Empty, "dance2", self.dance2_callback)

        # Timers
        self.timer = self.create_timer(self.publish_interval, self.timer_callback)

        # Command state
        self.current_request = None
        self.reset_state = False

        self.get_logger().info("Go2 Command Processor Node Started")

    def create_request(self, api_id, parameters=None):
        """Helper function to create a Request message."""
        req = Request()
        req.header.identity.api_id = api_id
        if parameters:
            req.parameter = parameters
        return req

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel messages. Converts the velocity commands into SportMode commands.
        """
        self.get_logger().info(f"Received cmd_vel: linear.x={msg.linear.x}, linear.y={msg.linear.y}, angular.z={msg.angular.z}")

        # Create a SportMode Move request
        req = self.create_request(
            api_id=1008,  # API ID for "Move"
            parameters=f'{{"x": {msg.linear.x}, "y": {msg.linear.y}, "z": {msg.angular.z}}}',
        )

        # Publish the request
        self.publisher.publish(req)
        self.current_request = req
        self.cmd_vel_counter = 0.0  # Reset the timeout counter

    def timer_callback(self):
        """
        Timer callback to handle periodic command publishing and timeout.
        """
        if self.cmd_vel_counter >= self.cmd_vel_timeout_duration:
            # Send Idle command if cmd_vel has timed out
            self.publish_idle_command()
        else:
            self.cmd_vel_counter += self.publish_interval

        # Handle reset state
        if self.reset_state:
            self.reset_state = False
            self.publish_idle_command()

    def publish_idle_command(self):
        """Publishes an Idle command."""
        req = self.create_request(api_id=0)  # API ID for "Idle"
        self.publisher.publish(req)
        self.get_logger().info("Published Idle command")

    # Service Callbacks
    def stand_up_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1004))  # StandUp
        self.reset_state = True
        self.get_logger().info("Stand Up command sent")
        return response

    def lay_down_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1005))  # StandDown
        self.reset_state = True
        self.get_logger().info("Lay Down command sent")
        return response

    def recover_stand_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1006))  # RecoveryStand
        self.reset_state = True
        self.get_logger().info("Recover Stand command sent")
        return response

    def damping_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1001))  # Damping
        self.reset_state = True
        self.get_logger().info("Damping command sent")
        return response

    def dance1_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1022))  # Dance1
        self.reset_state = True
        self.get_logger().info("Dance1 command sent")
        return response

    def dance2_callback(self, request, response):
        self.publisher.publish(self.create_request(api_id=1023))  # Dance2
        self.reset_state = True
        self.get_logger().info("Dance2 command sent")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Go2CmdProcessor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


