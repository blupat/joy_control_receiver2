#!/usr/bin/env python3
#
# =======================================================================
#   @file   joy_control_receiver.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pimouse_msgs.srv import PiMouseCmd


class JoyControlReceiver(Node):

    __slots__ = [
        '_joy_subscriber', '_srv_pimouse_cmd', '_is_on', '_is_run', '_is_face',
        '_sw_on_off', '_sw_run', '_sw_face', '_future']

    def __init__(self):
        super().__init__('joy_control_receiver')
        self._joy_subscriber = self.create_subscription(Joy, '/joy', self.receive_joy_callback, 1)
        self._srv_pimouse_cmd = self.create_client(PiMouseCmd, 'pimouse_cmd')
        self._is_on = False
        self._is_run = False
        self._is_face = False
        self._sw_on_off = 0
        self._sw_run = 0
        self._sw_face = 0
        self._future = None
        while not self._srv_pimouse_cmd.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting...')

    def receive_joy_callback(self, message):
        if self._future is not None:
            if not self._future.done():
                return
            self._future = None

        if (self._sw_on_off == 1) and (message.buttons[0] == 0):
            if self._is_on:
                self._is_on = False
                self._is_run = False
                self._is_face = False
            else:
                self._is_on = True
                self._is_run = False
                self._is_face = False
        elif (self._sw_run == 1) and (message.buttons[1] == 0):
            self._is_on = True
            self._is_run = True
            self._is_face = False
        elif (self._sw_face == 1) and (message.buttons[2] == 0):
            self._is_on = True
            self._is_run = False
            self._is_face = True
        self._sw_on_off = message.buttons[0]
        self._sw_run = message.buttons[1]
        self._sw_face = message.buttons[2]

        req = PiMouseCmd.Request()
        if self._is_on and (not self._is_run) and (not self._is_face):
            forward = message.axes[1] * 0.300
            rotation = 3.141592 * message.axes[2] * 90.0 / 180.0
            # self.get_logger().info('{0} [m/s], {1} [rad/s]'.format(forward, rotation))
            req.on = True
            req.run = False
            req.face = False
            req.forward = forward
            req.rotation = rotation
        elif self._is_on and self._is_run:
            # self.get_logger().info('Run')
            req.on = True
            req.run = True
            req.face = False
            req.forward = 0.0
            req.rotation = 0.0
        elif self._is_on and self._is_face:
            # self.get_logger().info('Face')
            req.on = True
            req.run = False
            req.face = True
            req.forward = 0.0
            req.rotation = 0.0
        else:
            # self.get_logger().info('Off')
            req.on = False
            req.run = False
            req.face = False
            req.forward = 0.0
            req.rotation = 0.0

        self._future = self._srv_pimouse_cmd.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    receiver = JoyControlReceiver()

    rclpy.spin(receiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
