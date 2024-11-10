#!/usr/bin/env python3
import rclpy
from Cython.Compiler.Naming import self_cname
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Bool
from rclpy.parameter import Parameter
from sympy.physics.units import speed
from turtlesim.srv import TeleportAbsolute


class TurtlePositionNode(Node):
    def __init__(self):
        super().__init__(node_name='Turtle_Position_Node', allow_undeclared_parameters=True,
                         start_parameter_services=True, automatically_declare_parameters_from_overrides=True)
        self.timer = None
        self.logger = self.get_logger()
        self._log_level = self.get_parameter_or('log_level', Parameter(name='log_level', value=10)).value
        self.logger.set_level(self._log_level)

        self.speed = self.get_parameter_or('turtle_speed', Parameter(name='turtle_speed', value=.3)).value
        self.speed_angular = 1.0
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.create_timer(.1, self.move_turtle)
        self.create_timer(1, self.update_speed)

        self.is_right_key_pressed = False
        self.right_key_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                              topic='/turtle/right',
                                                              callback=self.right_key_callback,
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.is_left_key_pressed = False
        self.left_key_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                              topic='/turtle/left',
                                                              callback=self.left_key_callback,
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.is_up_key_pressed = False
        self.up_key_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                              topic='/turtle/up',
                                                              callback=self.up_key_callback,
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.is_down_key_pressed = False
        self.down_key_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                              topic='/turtle/down',
                                                              callback=self.down_key_callback,
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.is_reset_key_pressed = False
        self.reset_key_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                              topic='/turtle/reset',
                                                              callback=self.reset_key_callback,
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.logger.info('Turtle Position Node has been started')

    def right_key_callback(self, msg: Bool):
        self.is_right_key_pressed = msg.data

    def left_key_callback(self, msg: Bool):
        self.is_left_key_pressed = msg.data

    def up_key_callback(self, msg: Bool):
        self.is_up_key_pressed = msg.data

    def down_key_callback(self, msg: Bool):
        self.is_down_key_pressed = msg.data

    def update_speed(self):
        new_speed = self.get_parameter_or('turtle_speed', Parameter(name='turtle_speed', value=.3)).value
        if new_speed != self.speed:
            self.logger.info(f'Speed updated to: {new_speed}')
        self.speed = new_speed

    def move_turtle(self):
        cmd_vel = Twist()
        if self.is_right_key_pressed and not self.is_left_key_pressed:
            cmd_vel.angular.z = -self.speed_angular
        elif self.is_left_key_pressed and not self.is_right_key_pressed:
            cmd_vel.angular.z = self.speed_angular

        if self.is_up_key_pressed and not self.is_down_key_pressed:
            cmd_vel.linear.x = self.speed
        elif self.is_down_key_pressed and not self.is_up_key_pressed:
            cmd_vel.linear.x = -self.speed


        self.publisher.publish(cmd_vel)

    def reset_key_callback(self, msg: Bool):
        if not msg.data:
            return

        self.call_teleport_service(5.5, 5.5, 0.0)

    def call_teleport_service(self, x: float, y: float, theta: float):
        # Create a client for the teleport service
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # Wait for the service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Service not available, waiting...')

        # Create a request object and set the parameters
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        # Send the request
        future = client.call_async(request)
        future.add_done_callback(self.teleport_callback)

    def teleport_callback(self, future):
        try:
            future.result()
            self.logger.info('Teleported successfully!')
        except Exception as e:
            self.logger.error(f'Failed to teleport: {e}')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TurtlePositionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        print(ex)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()