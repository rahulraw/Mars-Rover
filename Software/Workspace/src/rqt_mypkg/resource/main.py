# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'RoverUI_main.ui'
#
# Created: Wed Mar 11 00:11:34 2015
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(825, 595)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.pushButton = QtGui.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(720, 170, 85, 27))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.label_cam = QtGui.QLabel(self.centralwidget)
        self.label_cam.setGeometry(QtCore.QRect(20, 10, 106, 17))
        self.label_cam.setObjectName(_fromUtf8("label_cam"))
        self.label_orient = QtGui.QLabel(self.centralwidget)
        self.label_orient.setGeometry(QtCore.QRect(20, 270, 124, 17))
        self.label_orient.setObjectName(_fromUtf8("label_orient"))
        self.horizontalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(170, 100, 299, 153))
        self.horizontalLayoutWidget.setObjectName(_fromUtf8("horizontalLayoutWidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setMargin(0)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label_cam_zoom_out = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_cam_zoom_out.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.label_cam_zoom_out.setObjectName(_fromUtf8("label_cam_zoom_out"))
        self.gridLayout.addWidget(self.label_cam_zoom_out, 2, 1, 2, 1)
        self.tb_cam_xpos = QtGui.QLineEdit(self.horizontalLayoutWidget)
        self.tb_cam_xpos.setObjectName(_fromUtf8("tb_cam_xpos"))
        self.gridLayout.addWidget(self.tb_cam_xpos, 5, 0, 1, 2)
        self.pb_cam_right = QtGui.QPushButton(self.horizontalLayoutWidget)
        self.pb_cam_right.setObjectName(_fromUtf8("pb_cam_right"))
        self.gridLayout.addWidget(self.pb_cam_right, 1, 4, 2, 1)
        self.tb_cam_ypos = QtGui.QLineEdit(self.horizontalLayoutWidget)
        self.tb_cam_ypos.setObjectName(_fromUtf8("tb_cam_ypos"))
        self.gridLayout.addWidget(self.tb_cam_ypos, 5, 3, 1, 2)
        self.pb_cam_left = QtGui.QPushButton(self.horizontalLayoutWidget)
        self.pb_cam_left.setObjectName(_fromUtf8("pb_cam_left"))
        self.gridLayout.addWidget(self.pb_cam_left, 1, 0, 2, 1)
        self.pb_cam_down = QtGui.QPushButton(self.horizontalLayoutWidget)
        self.pb_cam_down.setObjectName(_fromUtf8("pb_cam_down"))
        self.gridLayout.addWidget(self.pb_cam_down, 3, 2, 1, 1)
        self.tb_cam_zpos = QtGui.QLineEdit(self.horizontalLayoutWidget)
        self.tb_cam_zpos.setObjectName(_fromUtf8("tb_cam_zpos"))
        self.gridLayout.addWidget(self.tb_cam_zpos, 5, 2, 1, 1)
        self.label_cam_zoom = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_cam_zoom.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cam_zoom.setObjectName(_fromUtf8("label_cam_zoom"))
        self.gridLayout.addWidget(self.label_cam_zoom, 4, 2, 1, 1)
        self.pb_cam_up = QtGui.QPushButton(self.horizontalLayoutWidget)
        self.pb_cam_up.setObjectName(_fromUtf8("pb_cam_up"))
        self.gridLayout.addWidget(self.pb_cam_up, 0, 2, 1, 1)
        self.scrollbar_cam_zoom = QtGui.QScrollBar(self.horizontalLayoutWidget)
        self.scrollbar_cam_zoom.setOrientation(QtCore.Qt.Horizontal)
        self.scrollbar_cam_zoom.setObjectName(_fromUtf8("scrollbar_cam_zoom"))
        self.gridLayout.addWidget(self.scrollbar_cam_zoom, 1, 1, 1, 3)
        self.label_cam_xpos = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_cam_xpos.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cam_xpos.setObjectName(_fromUtf8("label_cam_xpos"))
        self.gridLayout.addWidget(self.label_cam_xpos, 4, 0, 1, 2)
        self.label_cam_ypos = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_cam_ypos.setAlignment(QtCore.Qt.AlignCenter)
        self.label_cam_ypos.setObjectName(_fromUtf8("label_cam_ypos"))
        self.gridLayout.addWidget(self.label_cam_ypos, 4, 3, 1, 2)
        self.label_cam_zoom_in = QtGui.QLabel(self.horizontalLayoutWidget)
        self.label_cam_zoom_in.setAlignment(QtCore.Qt.AlignHCenter|QtCore.Qt.AlignTop)
        self.label_cam_zoom_in.setObjectName(_fromUtf8("label_cam_zoom_in"))
        self.gridLayout.addWidget(self.label_cam_zoom_in, 2, 3, 2, 1)
        self.horizontalLayout.addLayout(self.gridLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 825, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButton.setText(QtGui.QApplication.translate("MainWindow", "IMU Data", None, QtGui.QApplication.UnicodeUTF8))
        self.label_cam.setText(QtGui.QApplication.translate("MainWindow", "Camera Stream:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_orient.setText(QtGui.QApplication.translate("MainWindow", "Rover Orientation:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_cam_zoom_out.setText(QtGui.QApplication.translate("MainWindow", "-", None, QtGui.QApplication.UnicodeUTF8))
        self.tb_cam_xpos.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.pb_cam_right.setText(QtGui.QApplication.translate("MainWindow", "Right", None, QtGui.QApplication.UnicodeUTF8))
        self.tb_cam_ypos.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.pb_cam_left.setText(QtGui.QApplication.translate("MainWindow", "Left", None, QtGui.QApplication.UnicodeUTF8))
        self.pb_cam_down.setText(QtGui.QApplication.translate("MainWindow", "Down", None, QtGui.QApplication.UnicodeUTF8))
        self.tb_cam_zpos.setText(QtGui.QApplication.translate("MainWindow", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_cam_zoom.setText(QtGui.QApplication.translate("MainWindow", "zoom", None, QtGui.QApplication.UnicodeUTF8))
        self.pb_cam_up.setText(QtGui.QApplication.translate("MainWindow", "Up", None, QtGui.QApplication.UnicodeUTF8))
        self.label_cam_xpos.setText(QtGui.QApplication.translate("MainWindow", "x-pos", None, QtGui.QApplication.UnicodeUTF8))
        self.label_cam_ypos.setText(QtGui.QApplication.translate("MainWindow", "y-pos", None, QtGui.QApplication.UnicodeUTF8))
        self.label_cam_zoom_in.setText(QtGui.QApplication.translate("MainWindow", "+", None, QtGui.QApplication.UnicodeUTF8))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

