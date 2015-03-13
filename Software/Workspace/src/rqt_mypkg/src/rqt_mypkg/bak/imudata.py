import os
import rospy
import rospkg
import tf

from PyQt4 import QtCore, QtGui
from PyQt4.uic import loadUi
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from sensor_msgs.msg import Imu

try:  
    from PyQt4.QtCore import QString  
except ImportError:  
    # we are using Python3 so QString is not defined  
    QString = str 

class imuThread(QtCore.QThread):
    def __init__(self):
        QtCore.QThread.__init__(self)

    def run(self):
        print("imu thread run")
        rospy.Subscriber("/imu/data", Imu, self.callback)
        
    def callback(self, data):
        self.emit(QtCore.SIGNAL('updateImu(PyQt_PyObject)'), data)

class imu_dialog():
    def __init__(self):
        rp = rospkg.RosPack()
        imu_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource','RoverUI_imu.ui')
        self._dialog = loadUi(imu_file)
        # self.startImu()

    def startImu(self):
        self.imuT = imuThread()
        self._dialog.connect(self.imuT, QtCore.SIGNAL('updateImu(PyQt_PyObject)'), self.updateImu, Qt.QueuedConnection)
        self.imuT.start()

    def getEuler(self, data):
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

    def updateImu(self, data):
        self.getEuler(data)
        # print("roll = " + str(self.roll))
        # print("pitch = " + str(self.pitch))
        # print("yaw = " + str(self.yaw))
        # Quaternion values
        self._dialog.lineEdit_Quat_x.setText(QString(data.orientation.x))
        self._dialog.lineEdit_Quat_y.setText(QString(data.orientation.y))
        self._dialog.lineEdit_Quat_z.setText(QString(data.orientation.z))
        self._dialog.lineEdit_Quat_w.setText(QString(data.orientation.w))

        # Angular Velocity Values
        self._dialog.lineEdit_AV_x.setText(QString(data.angular_velocity.x))
        self._dialog.lineEdit_AV_y.setText(QString(data.angular_velocity.y))
        self._dialog.lineEdit_AV_z.setText(QString(data.angular_velocity.z))

        # Linear Acceleration Values
        self._dialog.lineEdit_LA_x.setText(QString(data.linear_acceleration.x))
        self._dialog.lineEdit_LA_y.setText(QString(data.linear_acceleration.y))
        self._dialog.lineEdit_LA_z.setText(QString(data.linear_acceleration.z))
        