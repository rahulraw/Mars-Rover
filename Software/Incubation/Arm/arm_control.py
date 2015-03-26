import math
import rospy
import jrk
from joystick_packages.msg import Controller

class JrkArm:
    def __init__(self, node_id):
        self.jrk = jrk.Jrk(node_id)
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

        self.arm1 = JrkArm("/dev/ttyACM0")
        self.controller = Controller()

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Controller, self.callback)
        self.rate = rospy.Rate(10)
        
    def get_direction(self, value):
        return int(0 if not value else value / abs(value))

    def callback(self, controller):
        self.arm1.set_direction(self.get_direction(controller.left_joy_y))

    def start(self):

        self.arm1.jrk.jrkGetErrorFlagsHalting()
        
        while not rospy.is_shutdown():
            self.arm1.run()
            self.rate.sleep()


if __name__=='__main__':
    args = ["RCValues", "ArmControl"]

    arm_control = ArmControl(args[0], args[1])
    arm_control.start()
