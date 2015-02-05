import rospy
import jrk
from std_msgs.msg import Int16MultiArray

class JrkArm:

    STOP = 0
    EXTEND = 1
    RETRACT = -1
    JRK_TARGET_OFFSET = 500
    JRK_TARGET_MIN = 0
    JRK_TARGET_MAX = 4095

    def __init__(self, jrkController):
        self.jrk = jrkController
        self.dir = JrkArm.STOP
        self.prevDir = JrkArm.STOP
        self.stopTarget = 0


class ArmControl:

    def __init__(self, topic = 'RCValues', node = "ArmControl"):

        self.topic = topic
        self.node = node

        self.jrkArm1 = JrkArm(jrk.Jrk("/dev/ttyACM0"))

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Int16MultiArray, self.callback)
        self.rate = rospy.Rate(5)
        #rospy.spin()

    def callback(self, data):

        jrk1_dir_raw = data.data[0]

        if jrk1_dir_raw > 45:
            arm_dir = JrkArm.EXTEND
        elif jrk1_dir_raw < -45:
            arm_dir = JrkArm.RETRACT
        else:
            arm_dir = JrkArm.STOP

        if arm_dir != self.jrkArm1.dir:
            self.jrkArm1.prevDir = self.jrkArm1.dir
            self.jrkArm1.dir = arm_dir

            if self.jrkArm1.dir == JrkArm.STOP:
                self.jrkArm1.stopTarget = self.jrkArm1.jrk.jrkGetFeedBack()


    def run_jrk(self, jrkArm):

        feedback = jrkArm.jrk.jrkGetFeedBack()

        if jrkArm.dir == JrkArm.EXTEND or jrkArm.dir == JrkArm.RETRACT:
            
            target =  feedback + jrkArm.dir * JrkArm.JRK_TARGET_OFFSET

            if target < JrkArm.JRK_TARGET_MIN:
                target = JrkArm.JRK_TARGET_MIN
            elif target > JrkArm.JRK_TARGET_MAX:
                target = JrkArm.JRK_TARGET_MAX

        else:
            target = jrkArm.stopTarget

        print("Dir:%2d   Feedback:%4d   Target:%4d" % (jrkArm.dir, feedback, target))
        jrkArm.jrk.jrkSetTarget(target)

    def start(self):

        self.jrkArm1.jrk.jrkGetErrorFlagsHalting()
        
        while not rospy.is_shutdown():

            self.run_jrk(self.jrkArm1)
            self.rate.sleep()

if __name__=='__main__':
    args = ["RCValues", "ArmControl"]

    arm_control = ArmControl(args[0], args[1])
    arm_control.start()
