import rospy
import jrk
from joystick_packages.msg import Controller

class JrkArm:

    JRK_TARGET_CONSTANT = 7
    JRK_TARGET_MIN = 100
    JRK_TARGET_MAX = 4000

    def __init__(self, node_id):
        self.jrk = jrk.Jrk(node_id)
        self.direction = 0
        self.target = self.input_from_target(self.jrk.jrkGetFeedBack())

    def contain(self, target):
        return max(
                self.JRK_TARGET_MIN, 
                min(self.JRK_TARGET_MAX, target)
            )

    def input_from_target(self, target):
        return self.contain(int(target * 1.142 - 88.782))
    
    def set_direction(self, direction):
        self.direction = direction

    def run(self):
        self.target = self.contain(self.target + self.direction * self.JRK_TARGET_CONSTANT)
        self.jrk.jrkSetTarget(self.target)


class ArmControl:
    def __init__(self, topic = 'RCValues', node = "ArmControl"):
        self.topic = topic
        self.node = node

        self.arm1 = JrkArm("/dev/ttyACM0")
        self.controller = Controller()

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Controller, self.callback)
        self.rate = rospy.Rate(40)
        
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
