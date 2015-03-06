#!usr/bin/env python
import time
import rospy

from joystick_packages.msg import Controller, ControllerRaw
from std_msgs.msg import Bool
from auto_shutdown import AutoShutdown

class Joystick:
    def __init__(self):
        self.topic = rospy.get_param('~topic', 'RCValues')
        self.node = rospy.get_param('~node', 'joystick') 
        self.pipe = open(rospy.get_param('~input_id', '/dev/input/js1'), 'r')
        self.pconstant = 0.2
        self.controller = Controller()
        self.set_buttons = [self.set_square, self.set_x, self.set_circle, self.set_triangle, self.set_left_bumper, self.set_right_bumper, self.set_left_trigger, self.set_right_trigger, self.set_share, self.set_options, self.set_left_stick, self.set_right_stick, self.set_killswitch, self.set_touch_button]
        self.set_axis = [self.set_left_joy_x, self.set_left_joy_y, self.set_right_joy_x, self.set_left_brake, self.set_right_brake, self.set_right_joy_y, self.set_pad_x, self.set_pad_y]
        self.current_time = 0
        self.new_time = 0
    
        rospy.init_node(self.node, anonymous = True)
        self.pub = rospy.Publisher(self.topic, Controller, queue_size = 10)

        self.auto_shutdown = AutoShutdown(10*60)
        self.auto_shutdown.start()

    def start(self):

        brake_value = 0

        msg = []
        killswitch = 0
        
        while not rospy.is_shutdown():
            for char in self.pipe.read(1):
                self.auto_shutdown.updateTime()
                msg += [ord(char)]
                
                if len(msg) == 8:
                    controller_raw = ControllerRaw()
                    controller_raw.type = msg[6]
                    controller_raw.number = msg[7]

                    if controller_raw.type == 1:
                        controller_raw.value = msg[4]
                        self.set_buttons[controller_raw.number](controller_raw.value)

                    elif controller_raw.type == 2:

                        controller_raw.value = msg[5]

                        # The values go from 255 - 0 as you go more left.
                        # This is unintuitive, so we correct by subtracting
                        # 255.
                        if msg[5] > 127: 
                            controller_raw.value = msg[5] - 255

                        # Up is a negative value typically, which is unintuitive
                        # Up axes are the odd axes, so we multiply value by -1 
                        # if axis is odd.
                        if controller_raw.number % 2:
                            controller_raw.value *= -1
 
                        controller_raw.value = int(controller_raw.value * 90 / 127)
                        self.set_axis[controller_raw.number](controller_raw.value)

                    # Publish message
                    self.pub.publish(self.controller)
                    msg = []

    def set_square(self, value):
        if value == 1:
            self.controller.square = 1 - self.controller.square

    def set_x(self, value):
        if value == 1:
            self.controller.x = 1 - self.controller.x

    def set_circle(self, value):
        if value == 1:
            self.controller.circle = 1 - self.controller.circle

    def set_triangle(self, value):
        if value == 1:
            self.controller.triangle = 1 - self.controller.triangle

    def set_left_bumper(self, value):
        if value == 1:
            self.controller.mode = (self.controller.mode - 1) % 4

    def set_right_bumper(self, value):
        if value == 1:
            self.controller.mode = (self.controller.mode + 1) % 4

    def set_left_trigger(self, value):
        self.controller.left_trigger = value

    def set_right_trigger(self, value):
        self.controller.right_trigger = value

    def set_share(self, value):
        self.controller.share = value

    def set_options(self, value):
        self.controller.options = value

    def set_left_stick(self, value):
        self.controller.left_stick = value

    def set_right_stick(self, value):
        self.controller.right_stick = value

    def set_killswitch(self, value):
        if value == 1:
            self.controller.killswitch = 1 - self.controller.killswitch

    def set_touch_button(self, value):
        self.controller.touch_button = value

    def set_left_joy_x(self, value):
        self.controller.left_joy_x = value

    def set_left_joy_y(self, value):
        self.controller.left_joy_y = value

    def set_right_joy_x(self, value):
        self.controller.right_joy_x = value

    def set_left_brake(self, value):
        self.controller.left_brake = value

    def set_right_brake(self, value):
        self.controller.right_brake = value

    def set_right_joy_y(self, value):
        self.controller.right_joy_y = value

    def set_pad_x(self, value):
        self.controller.pad_x = value

    def set_pad_y(self, value):
        self.controller.pad_y = value
        
if __name__ == '__main__':
    try:
        joystick = Joystick()
        joystick.start() 
    except rospy.ROSInterruptException: pass
