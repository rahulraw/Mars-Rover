#!usr/bin/env python

import rospy

from joystick_packages.msg import JoystickMsg
from std_msgs.msg import Int16MultiArray

class Joystick:
    def __init__(self):
        self.topic = rospy.get_param('~topic', 'RCValues')
        self.node = rospy.get_param('~node', 'joystick') 
        self.pipe = open(rospy.get_param('~input_id', '/dev/input/js1'), 'r')
        self.pconstant = 0.2

    def start(self):
        rospy.init_node(self.node, anonymous = True)
        self.pub = rospy.Publisher(self.topic, Int16MultiArray, queue_size = 10)
        joy_msg = Int16MultiArray()
        joy_msg.data = [0] * 4

        throttle_value = 0;

        msg = []
        killswitch = 0
        while not rospy.is_shutdown():
            for char in self.pipe.read(1):
                msg += [ord(char)]
                
                if len(msg) == 8:
                    joystick_message = JoystickMsg()
                    joystick_message.type = msg[6]
                    joystick_message.number = msg[7]

                    if joystick_message.type == 1:
                        joystick_message.value = msg[4]

                        if joystick_message.number == 7:
                            joy_msg.data[3] = joystick_message.value 

                        if joystick_message.number == 9:
                            killswitch += 1
                            if killswitch == 2:
                                killswitch = 0
                                joy_msg.data[2] = 1 - joy_msg.data[2]

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
                        
                        print(joystick_message.number)
                        print(joystick_message.value)
                        if joystick_message.number == 0:
                            # joy_msg.data[0] = joystick_message.value
                            pass
                       
                        elif joystick_message.number == 3:
                            joy_msg.data[1] = joystick_message.value

                        elif joystick_message.number == 5:
                            throttle_value = joystick_message.value
                        
                        if throttle_value > 0:   
                            self.pconstant = self.pconstant/1.1
                            joy_msg.data[0] = min(90, joy_msg.data[0] + (self.pconstant * throttle _value))
                        
                        elif throttle_value < 0:
                            joy_msg.data[0] = max(-90, joy_msg.data[0] - (self.pconstant * throttle _value))
                            self.pconstant = self.pconstant/1.1
                        
                        elif (throttle_value == 0):
                            joy_msg.data[0] = 0
                            self.pconstant = 0.2


                    # Publish message
                    self.pub.publish(joy_msg)
                    msg = []
        
if __name__ == '__main__':
    try:
        joystick = Joystick()
        joystick.start() 
    except rospy.ROSInterruptException: pass
