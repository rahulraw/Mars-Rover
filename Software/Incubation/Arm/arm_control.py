import rospy
import jrk
from std_msgs.msg import Int16MultiArray


class ArmControl:

    STOP = 0
    EXTEND = 1
    RETRACT = -1
    JRK_TARGET_OFFSET = 500
    JRK_TARGET_MIN = 0
    JRK_TARGET_MAX = 4095

    def __init__(self, topic = 'RCValues', node = "ArmControl"):

        self.topic = topic
        self.node = node

        self.jrk1 = jrk.Jrk("/dev/ttyACM0")
        self.jrk1_dir = self.STOP

        rospy.init_node(self.node, anonymous=True)
        rospy.Subscriber(self.topic, Int16MultiArray, self.callback)
        self.rate = rospy.Rate(5)
        #rospy.spin()

    def callback(self, data):

        jrk1_dir_raw = data.data[0]

        if jrk1_dir_raw > 45:
            self.jrk1_dir = self.EXTEND
        elif jrk1_dir_raw < -45:
            self.jrk1_dir = self.RETRACT
        else:
            self.jrk1_dir = self.STOP

    def run_jrk(self, jrk, jrk_dir):

        feedback = jrk.jrkGetFeedBack()
        target = feedback

        if jrk_dir == self.EXTEND or jrk_dir == self.RETRACT:
            
            target += jrk_dir * self.JRK_TARGET_OFFSET

            if target < self.JRK_TARGET_MIN:
                target = self.JRK_TARGET_MIN
            elif target > self.JRK_TARGET_MAX:
                target = self.JRK_TARGET_MAX

        print("Dir:%2d   Feedback:%4d   Target:%4d" % (jrk_dir, feedback, target))
        jrk.jrkSetTarget(target)

    def start(self):

        self.jrk1.jrkGetErrorFlagsHalting()
        
        while not rospy.is_shutdown():

            self.run_jrk(self.jrk1, self.jrk1_dir)
            self.rate.sleep()

if __name__=='__main__':
    args = ["RCValues", "ArmControl"]

    arm_control = ArmControl(args[0], args[1])
    arm_control.start()
