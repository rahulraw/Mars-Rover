#!/usr/bin/python

import rospy
from joystick_packages.msg import Joystick as Msg, ControllerRaw
from std_msgs.msg import Int16MultiArray

class Joystick:
    def __init__(self):
        self.topic = rospy.get_param('topic', 'joystick1')
        self.node = rospy.get_param('node', 'joystick1')
        self.pipe = open(rospy.get_param('inputid', '/dev/input/js0'), 'r')
        self.pconstant = 0.2
        self.joy_msg = Msg()
        self.set_buttons = [self.set_trigger, self.set_b2, self.set_b3, self.set_b4, self.set_b5, self.set_b6, self.set_b7, self.set_b8, self.set_b9, self.set_b10, self.set_b11]
        self.set_axis = [self.set_main_joy_x, self.set_main_joy_y, self.set_bottom_toggle_y]

    def start(self):
        rospy.init_node(self.node, anonymous = True)
        self.pub = rospy.Publisher(self.topic, Msg, queue_size = 10)

        brake_value = 0

        msg = []
        killswitch = 0
        while not rospy.is_shutdown():
            for char in self.pipe.read(1):
                msg += [ord(char)]
                
                if len(msg) == 8:
                    joystick_message = ControllerRaw()
                    joystick_message.type = msg[6]
                    joystick_message.number = msg[7]

                    if joystick_message.type == 1:
                        joystick_message.value = msg[4]
                        self.set_buttons[joystick_message.number](joystick_message.value)

                    elif joystick_message.type == 2:

                        joystick_message.value = msg[5]

                        # The values go from 255 - 0 as you go more left.
                        # This is unintuitive, so we correct by subtracting
                        # 255.
                        if msg[5] > 127: 
                            joystick_message.value = msg[5] - 255

                        # Up is a negative value typically, which is unintuitive
                        # Up axes are the odd axes, so we multiply value by -1 
                        # if axis is odd.
                        if joystick_message.number % 2:
                            joystick_message.value *= -1
 
                        joystick_message.value = int(joystick_message.value * 90 / 127)
                        self.set_axis[joystick_message.number](joystick_message.value)

                    # Publish message
                    self.pub.publish(self.joy_msg)
                    msg = []

    def set_trigger(self, value):
        self.joy_msg.trigger = value

    def set_b2(self, value):
        self.joy_msg.b2 = value

    def set_b3(self, value):
        self.joy_msg.b3 = value

    def set_b4(self, value):
        self.joy_msg.b4 = value

    def set_b5(self, value):
        self.joy_msg.b5 = value

    def set_b6(self, value):
        self.joy_msg.b6 = value

    def set_b7(self, value):
        self.joy_msg.b7 = value

    def set_b8(self, value):
        if value == 1:
            self.joy_msg.b8 = 1 - self.joy_msg.b8

    def set_b9(self, value):
        self.joy_msg.b9 = value

    def set_b10(self, value):
        self.joy_msg.b10 = value

    def set_b11(self, value):
        self.joy_msg.b11 = value

    def set_main_joy_x(self, value):
        self.joy_msg.main_joy_x = value

    def set_main_joy_y(self, value):
        self.joy_msg.main_joy_y = value

    def set_bottom_toggle_y(self, value):
        self.joy_msg.bottom_toggle_y = value
        
if __name__ == '__main__':
    try:
        joystick = Joystick()
        joystick.start() 
    except rospy.ROSInterruptException: pass
