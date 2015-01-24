#!/usr/bin/env python

#http://ubuntuforums.org/archive/index.php/t-1695571.html#

import glib
import pyudev
from pyudev import Context, Device
import pyinotify
import sys
import os


context = Context()

roboclawfr_connected = False 
roboclawfl_connected = False
roboclawbr_connected = False
roboclawbl_connected = False
roboclaws_connected = False

while not roboclaws_connected:
    for device in context.list_devices(driver='usb'):
        if device.sysname =="roboclawfl" and not roboclawfl_connected:  
            roboclaw_add(device.sysname)
        if device.sysname =="roboclawfr" and not roboclawfr_connected: 
            roboclaw_add(device.sysname)
        if device.sysname =="roboclawbl" and not roboclawbl_connected: 
            roboclaw_add(device.sysname)
        if device.sysname =="roboclawbr" and not roboclawbr_connected: 
            roboclaw_add(device.sysname)
    if roboclawfl_connected and roboclawfr_connecte and roboclawbl_connected and roboclawbr_connected:
        roboclaws_connected = True

os.system("roslaunch Workspace startup.launch")

def roboclaw_add(roboclaw_name):
    n = pyinotify.Notification("USB Device Added", "%s is now connected to your system" % (roboclaw_name))
    n.show()
    if(roboclaw_name =="roboclawfl"):
       roboclawfr_connected = True
    if(roboclaw_name =="roboclawfr"):
       roboclawfl_connected = True
    if(roboclaw_name =="roboclawbl"):
       roboclawbl_connected = True
    if(roboclaw_name =="roboclawbr"):
       roboclawbr_connected = True





   #def event(client, action, device, user_data):
   #    device_vendor = device.get_property("ID_VENDOR_ENC")
   #    device_model = device.get_property("ID_MODEL_ENC")
   #    if action == "add":
   #        n = pyinotify.Notification("USB Device Added", "%s %s is now connected to your system" % (device_vendor, device_model))
   #        n.show()
   #        #start roslaunch.
   #        os.system("roslaunch Workspace startup.launch")
   #    
   #    #Don't need this elif statement since we only care when roboclaws connect.
   #    elif action == "remove":
   #        n = pyinotify.Notification("USB Device Removed", "%s %s has been ""disconnected from your system" %( applidevice_vendor, device_model))
   #        n.show()

   #    #Could keep this.
   #    if not pyinotify.init("USB Device Notifier"):
   #        sys.exit("Couldn't connect to the notification daemon!")


   #client = pyudev.Client(["/dev/roboclawfl"])
   #client.connect("uevent", event, None)

   #loop = glib.MainLoop()
   #loop.run()
