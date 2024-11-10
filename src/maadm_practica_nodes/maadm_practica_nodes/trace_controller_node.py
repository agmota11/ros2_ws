#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Bool
from rclpy.parameter import Parameter
from turtlesim.srv import SetPen
from std_srvs.srv import Empty


class TraceControllerNode(Node):
    def __init__(self):
        super().__init__(node_name='Trace_Controller_Node', allow_undeclared_parameters=True,
                         start_parameter_services=True, automatically_declare_parameters_from_overrides=True)
        self.timer = None
        self.logger = self.get_logger()
        self._log_level = self.get_parameter_or('log_level', Parameter(name='log_level', value=10)).value
        self.logger.set_level(self._log_level)


        self.is_trace_enable_key_pressed = False
        self.trace_enabled_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                                        topic='/turtle/trace/enable',
                                                                        callback=self.trace_enable_key_callback,
                                                                        qos_profile=HistoryPolicy.KEEP_LAST)

        self.is_clean_key_pressed = False
        self.clean_key_pressed_publisher = self.create_subscription(msg_type=Bool,
                                                                    topic='/turtle/trace/clear',
                                                                    callback=self.clean_key_callback,
                                                                    qos_profile=HistoryPolicy.KEEP_LAST)
        self.pen_width = 3
        self.set_pen(0)
        self.is_trace_disabled = False
        self.logger.info('Trace controller Node has been started')

    def trace_enable_key_callback(self, msg: Bool):
        if not self.is_trace_enable_key_pressed and msg.data:
            self.is_trace_enable_key_pressed = True
            self.set_pen(0 if self.is_trace_disabled else 1)
            self.is_trace_disabled = not self.is_trace_disabled

        if self.is_trace_enable_key_pressed and not msg.data:
            self.is_trace_enable_key_pressed = False

    def clean_key_callback(self, msg: Bool):
        if not msg.data:
            return

        self.call_clear_service()


    def set_pen(self, value: int):
        client = self.create_client(SetPen, '/turtle1/set_pen')

        while not client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Service not available, waiting...')

        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = self.pen_width
        request.off = value
        future = client.call_async(request)
        future.add_done_callback(self.set_pen_callback)


    def call_clear_service(self):
        client = self.create_client(Empty, '/clear')

        while not client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Service not available, waiting...')

        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(self.clear_callback)

    def set_pen_callback(self, future):
        try:
            future.result()
            self.logger.info('Turtle trace cleared!')
        except Exception as e:
            self.logger.error(f'Failed to clear trace: {e}')


    def clear_callback(self, future):
        try:
            future.result()
            self.logger.info('Turtle trace cleared!')
        except Exception as e:
            self.logger.error(f'Failed to clear trace: {e}')


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TraceControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        print(ex)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()