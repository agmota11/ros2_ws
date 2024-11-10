#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Bool
from rclpy.parameter import Parameter
from pynput import keyboard



class KeyboardListenerNode(Node):
    def __init__(self):
        super().__init__(node_name='Keyboard_Listener_Node', allow_undeclared_parameters=True,
                         start_parameter_services=True, automatically_declare_parameters_from_overrides=True)
        self.timer = None
        self.logger = self.get_logger()
        self._log_level = self.get_parameter_or('log_level', Parameter(name='log_level', value=10)).value
        self.logger.set_level(self._log_level)
        self.keyboard_start()
        self.pressed_keys = set()
        timer_period = self.get_parameter_or('freq', Parameter(name='freq', value=10)).value
        self.update_timer = self.create_timer(1 / timer_period, self.update)

        self.right_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/right',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.left_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/left',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.up_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/up',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.down_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/down',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)


        self.space_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/trace/enable',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)


        self.clear_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/trace/clear',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)


        self.reset_key_pressed_publisher = self.create_publisher(msg_type=Bool,
                                                              topic='/turtle/reset',
                                                              qos_profile=HistoryPolicy.KEEP_LAST)

        self.logger.info('Keyboard Listener Node has been started')

    def keyboard_start(self):
        lis = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        lis.start()  # start to listen on a separate thread

    def on_press(self, key):
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys

        self.pressed_keys.add(k)

    def on_release(self, key):
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys

        self.pressed_keys.remove(k)

    def update(self):
        self.right_key_pressed_publisher.publish(Bool(data='d' in self.pressed_keys))
        self.left_key_pressed_publisher.publish(Bool(data='a' in self.pressed_keys))
        self.up_key_pressed_publisher.publish(Bool(data='w' in self.pressed_keys))
        self.down_key_pressed_publisher.publish(Bool(data='s' in self.pressed_keys))
        self.space_key_pressed_publisher.publish(Bool(data='space' in self.pressed_keys))
        self.clear_key_pressed_publisher.publish(Bool(data='c' in self.pressed_keys))
        self.reset_key_pressed_publisher.publish(Bool(data='r' in self.pressed_keys))

def main(args=None):
    try:
        rclpy.init(args=args)
        node = KeyboardListenerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        print(ex)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()