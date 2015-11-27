#!/usr/bin/python
import naoqi
import sys
from naoqi import ALProxy
memory = ALProxy("ALMemory","127.0.0.1",9559)
print str(sys.argv[1]) + " " + str(memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value"))
