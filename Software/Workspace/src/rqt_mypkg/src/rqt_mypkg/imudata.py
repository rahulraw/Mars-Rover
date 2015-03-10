import os
import rospy
import rospkg
import rosbag

from qt_gui.plugin import Plugin

from PyQt4 import QtCore, QtGui
from PyQt4.uic import loadUi
from QtCore import Qt
from QtGui import QDialog, QWidget, QMainWindow, QListWidget

from sensor_msgs.msg import Imu

class imu_dialog():
    def __init__(self):
        rp = rospkg.RosPack()
        imu_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource','RoverUI_imu.ui')
        self._dialog = loadUi(imu_file)

    def updateImu(self, data):
        #self._dialog.label.setText(str(data.orientation.x))
        # Quaternion values
        self._dialog.lineEdit_Quat_x.setText(str(data.orientation.x))
        self._dialog.lineEdit_Quat_y.setText(str(data.orientation.y))
        self._dialog.lineEdit_Quat_z.setText(str(data.orientation.z))
        self._dialog.lineEdit_Quat_w.setText(str(data.orientation.w))

        # Angular Velocity Values
        self._dialog.lineEdit_AV_x.setText(str(data.angular_velocity.x))
        self._dialog.lineEdit_AV_y.setText(str(data.angular_velocity.y))
        self._dialog.lineEdit_AV_z.setText(str(data.angular_velocity.z))

        # Linear Acceleration Values
        self._dialog.lineEdit_LA_x.setText(str(data.linear_acceleration.x))
        self._dialog.lineEdit_LA_y.setText(str(data.linear_acceleration.y))
        self._dialog.lineEdit_LA_z.setText(str(data.linear_acceleration.z))
