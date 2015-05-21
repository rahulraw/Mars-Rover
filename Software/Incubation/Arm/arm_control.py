import math
import rospy
import jrk
from joystick_packages.msg import Joystick

class JrkArm:
    def __init__(self, node_id):
        self.jrk = jrk.Jrk("/dev/stick")
        self.direction = 0
        self.stopped = True
        self.jrk.jrkMotorStop()
    
    def set_direction(self, direction):
        self.direction = direction

    def run(self):
        if not self.direction and not self.stopped:
            self.stopped = True
            self.jrk.jrkMotorStop()
        elif self.direction:
            self.stopped = False
            target = int(2000 + 2000 * self.direction)
            self.jrk.jrkSetTarget(target)

class ArmControl:
    def __init__(self, topic = 'RCValues', node = "ArmControl"):
        self.topic = topic
        self.node = node

        self.boom = JrkArm("/dev/boom")
        self.stick = JrkArm("/dev/stick")

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber("joystick1", Joystick, self.callback1)
        rospy.Subscriber("joystick2", Joystick, self.callback2)
        self.rate = rospy.Rate(10)
        
    def get_direction(self, value):
        return int(0 if not value else value / abs(value))

    def callback1(self, controller):
        self.boom.set_direction(self.get_direction(controller.main_joy_y))

    def callback2(self, controller):
        self.stick.set_direction(self.get_direction(controller.main_joy_y))

    def start(self):

        self.boom.jrk.jrkGetErrorFlagsHalting()
        self.stick.jrk.jrkGetErrorFlagsHalting()
        
        while not rospy.is_shutdown():
            self.boom.run()
            self.stick.run()
            self.rate.sleep()


if __name__=='__main__':
    args = ["RCValues", "ArmControl"]

    arm_control = ArmControl(args[0], args[1])
    arm_control.start()
