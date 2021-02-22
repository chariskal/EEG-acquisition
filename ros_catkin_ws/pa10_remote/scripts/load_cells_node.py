#! /usr/bin/python

#Basic imports
import sys
from time import sleep
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetException
from Phidgets.Devices.Bridge import Bridge, BridgeGain
import numpy as np
#ROS Imports
import roslib 
import rospy
from sensor_msgs.msg import ChannelFloat32

#Vector to store data
a = np.zeros((4,1), dtype=float)

#Initialize Counter 
counter = 0.0
suma0 = 0.0
suma1 = 0.0
suma2 = 0.0
suma3 = 0.0
endcycle = 5000

#Initiate ROS Node
rospy.init_node('load_cells_node')
pub = rospy.Publisher("/load_cells/data", ChannelFloat32)

loadcellData = ChannelFloat32()

try:
    bridge = Bridge()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    exit(1)

#Information Display Function
def displayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (bridge.isAttached(), bridge.getDeviceName(), bridge.getSerialNum(), bridge.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of bridge inputs: %i" % (bridge.getInputCount()))
    print("Data Rate Max: %d" % (bridge.getDataRateMax()))
    print("Data Rate Min: %d" % (bridge.getDataRateMin()))
    print("Input Value Max: %d" % (bridge.getBridgeMax(0)))
    print("Input Value Min: %d" % (bridge.getBridgeMin(0)))

#Event Handler Callback Functions
def BridgeAttached(e):
    attached = e.device
    print("Bridge %i Attached!" % (attached.getSerialNum()))

def BridgeDetached(e):
    detached = e.device
    print("Bridge %i Detached!" % (detached.getSerialNum()))

def BridgeError(e):
    try:
        source = e.device
        print("Bridge %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))


def BridgeData(e):
    global loadcellData
    global counter
    global suma0
    global suma1
    global suma2
    global suma3
    
    source = e.device
    a[e.index] = e.value
  
    if counter < endcycle:
       suma0 = suma0 + a[0]
       suma1 = suma1 + a[1]
       suma2 = suma2 + a[2]
       suma3 = suma3 + a[3]
       counter = counter + 1.0
       print "CALIBRATING...", counter
       if counter == endcycle:
          print "CALIBRATION COMPLETED!","SYSTEM IN USE!"
          print "PRESS ENTER TO TERMINATE"
       
    off0 = suma0/counter
    off1 = suma1/counter
    off2 = suma2/counter
    off3 = suma3/counter

    if counter == endcycle:
       #Filling Data Msg
       loadcellData.values = list(a)
       loadcellData.values[0] = loadcellData.values[0] - off0
       loadcellData.values[1] = loadcellData.values[1] - off1
       loadcellData.values[2] = loadcellData.values[2] - off2
       loadcellData.values[3] = loadcellData.values[3] - off3
       
       #Uncomment when use with PA10!!!!!!!!
       #med = (loadcellData.values[0]+loadcellData.values[1]+loadcellData.values[2]+loadcellData.values[3])/4

       med = (loadcellData.values[0]+loadcellData.values[1]+loadcellData.values[2]+loadcellData.values[3])*80.0
       
       loadcellData.values.append(med)
        
       #Publish Data
       sleep(0.01)
       pub.publish(loadcellData)
       
    #print("Bridge %i: Input %i: %f" % (source.getSerialNum(), e.index, e.value))


#Main Program Code
try:
    bridge.setOnAttachHandler(BridgeAttached)
    bridge.setOnDetachHandler(BridgeDetached)
    bridge.setOnErrorhandler(BridgeError)
    bridge.setOnBridgeDataHandler(BridgeData)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Opening phidget object....")

try:
    bridge.openPhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Waiting for attach....")

try:
    bridge.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        bridge.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)
else:
    displayDeviceInfo()

try:
    print("Set data rate to 8ms ...")
    bridge.setDataRate(16)
    sleep(2)

    print("Set Gain to 8...")
    bridge.setGain(0, BridgeGain.PHIDGET_BRIDGE_GAIN_8)
    sleep(2)

    print("Enable the Bridge input for reading data...")
    bridge.setEnabled(0, True)
    bridge.setEnabled(1, True)
    bridge.setEnabled(2, True)
    bridge.setEnabled(3, True)
    sleep(2)

except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        bridge.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)

print("Press Enter to quit....")

chr = sys.stdin.read(1)

print("Closing...")

try:
    print("Disable the Bridge input for reading data...")
    bridge.setEnabled(0, False)
    bridge.setEnabled(1, False)
    bridge.setEnabled(2, False)
    bridge.setEnabled(3, False)
    sleep(2)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        bridge.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)

try:
    bridge.closePhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Done.")
exit(0)
