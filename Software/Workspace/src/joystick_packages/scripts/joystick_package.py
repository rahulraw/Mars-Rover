#!usr/bin/env python

import rospy

from joystick_packages.msg import JoystickMsg

class Joystick:
    def __init__(self):
        self.topic = rospy.get_param('~topic', 'input_stream')
        self.node = rospy.get_param('~node', 'joystick') 
        self.pipe = open(rospy.get_param('~input_id', '/dev/input/js0'), 'r')

    def start(self):
        rospy.init_node(self.node, anonymous = True)
        self.pub = rospy.Publisher(self.topic, JoystickMsg, queue_size = 10)

        msg = []
        while not rospy.is_shutdown():
            for char in self.pipe.read(1):
                msg += [ord(char)]

                if len(msg) == 8:
                    joystick_message = JoystickMsg()
                    joystick_message.type = msg[6]
                    joystick_message.number = msg[7]

                    if joystick_message.type == 1:
                        joystick_message.value = msg[4]

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
                    
                    # Publish message
                    self.pub.publish(joystick_message)
                    msg = []
        
if __name__ == '__main__':
    try:
        joystick = Joystick()
        joystick.start() 
    except rospy.ROSInterruptException: pass
