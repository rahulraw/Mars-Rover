#!usr/bin/env python

import rospy

from joystick_packages.msg import JoystickMsg, JoystickMsgRaw
from std_msgs.msg import Int16MultiArray

class Joystick:
    def __init__(self):
        self.topic = rospy.get_param('~topic', 'RCValues')
        self.node = rospy.get_param('~node', 'joystick') 
        self.pipe = open(rospy.get_param('~input_id', '/dev/input/js0'), 'r')
        self.pconstant = 0.2
        self.joy_msg = JoystickMsg()
        self.set_buttons = [self.set_square, self.set_x, self.set_circle, self.set_triangle, self.set_left_bumper, self.set_right_bumper, self.set_left_trigger, self.set_right_trigger, self.set_share, self.set_options, self.set_left_stick, self.set_right_stick, self.set_killswitch, self.set_touch_button]
        self.set_axis = [self.set_left_joy_x, self.set_left_joy_y, self.set_right_joy_x, self.set_left_brake, self.set_right_brake, self.set_right_joy_y, self.set_pad_x, self.set_pad_y]

    def start(self):
        rospy.init_node(self.node, anonymous = True)
        self.pub = rospy.Publisher(self.topic, JoystickMsg, queue_size = 10)

        brake_value = 0

        msg = []
        killswitch = 0
        while not rospy.is_shutdown():
            for char in self.pipe.read(1):
                msg += [ord(char)]
                
                if len(msg) == 8:
                    joystick_message = JoystickMsgRaw()
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

    def set_square(self, value):
        if value == 1:
            self.joy_msg.square = 1 - self.joy_msg.square

    def set_x(self, value):
        if value == 1:
            self.joy_msg.x = 1 - self.joy_msg.x

    def set_circle(self, value):
        if value == 1:
            self.joy_msg.circle = 1 - self.joy_msg.circle

    def set_triangle(self, value):
        if value == 1:
            self.joy_msg.triangle = 1 - self.joy_msg.triangle

    def set_left_bumper(self, value):
        self.joy_msg.left_bumper = (self.joy_msg.left_bumper + value) % 128

    def set_right_bumper(self, value):
        self.joy_msg.right_bumper = (self.joy_msg.right_bumper + value) % 128

    def set_left_trigger(self, value):
        self.joy_msg.left_trigger = value

    def set_right_trigger(self, value):
        self.joy_msg.right_trigger = value

    def set_share(self, value):
        self.joy_msg.share = value

    def set_options(self, value):
        self.joy_msg.options = value

    def set_left_stick(self, value):
        pass

    def set_right_stick(self, value):
        pass

    def set_killswitch(self, value):
        if value == 1:
            self.joy_msg.killswitch = 1 - self.joy_msg.killswitch

    def set_touch_button(self, value):
        pass

    def set_left_joy_x(self, value):
        self.joy_msg.left_joy_x = value

    def set_left_joy_y(self, value):
        self.joy_msg.left_joy_y = value

    def set_right_joy_x(self, value):
        self.joy_msg.right_joy_x = value

    def set_left_brake(self, value):
        self.joy_msg.left_brake = value

    def set_right_brake(self, value):
        self.joy_msg.right_brake = value

    def set_right_joy_y(self, value):
        self.joy_msg.right_joy_y = value

    def set_pad_x(self, value):
        pass

    def set_pad_y(self, value):
        pass
        
if __name__ == '__main__':
    try:
        joystick = Joystick()
        joystick.start() 
    except rospy.ROSInterruptException: pass
