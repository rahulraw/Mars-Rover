#!/usr/bin/env python
import rospy

import sys
import traceback

from PyQt4 import QtCore, QtGui
import imuGUI
from sensor_msgs.msg import Imu

class imuDiaglog(QtGui.QDialog, imuGUI.Ui_Dialog):
    def __init__(self, parent=None):
        super(imuDiaglog, self).__init__(parent)
        self.setupUi(self)

    def our_str(self, data):
        return str(data)

    def updateIMUdata(self, data):
        # Quaternion values
        self.lineEdit_Quat_x.setText(self.our_str(data.orientation.x))
        self.lineEdit_Quat_y.setText(self.our_str(data.orientation.y))
        self.lineEdit_Quat_z.setText(self.our_str(data.orientation.z))
        self.lineEdit_Quat_w.setText(self.our_str(data.orientation.w))

        # Angular Velocity Values
        self.lineEdit_AV_x.setText(self.our_str(data.angular_velocity.x))
        self.lineEdit_AV_y.setText(self.our_str(data.angular_velocity.y))
        self.lineEdit_AV_z.setText(self.our_str(data.angular_velocity.z))

        # Linear Acceleration Values
        self.lineEdit_LA_x.setText(self.our_str(data.linear_acceleration.x))
        self.lineEdit_LA_y.setText(self.our_str(data.linear_acceleration.y))
        self.lineEdit_LA_z.setText(self.our_str(data.linear_acceleration.z))

class imuData:
    def __init__(self):
        self.app = QtGui.QApplication(sys.argv)
        self.form = imuDiaglog()
        self.form.show()

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/imu/data", Imu, self.callback)

        self.app.exec_()

    def callback(self, data):
        self.form.updateIMUdata(data)
    #     rospy.loginfo("\nOrientation:\n%s", data.orientation.x)
    
    def listener(self):
        while not rospy.is_shutdown():
            rospy.spin()
            self.rate.sleep()
    #         spin() simply keeps python from exiting until this node is stopped
    #         rospy.spin()

if __name__ == '__main__':
    imu = imuData()
    imu.listener()
