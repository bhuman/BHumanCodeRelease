#!/usr/bin/python
import naoqi, sys, os
from naoqi import ALProxy
memory = ALProxy("ALMemory","127.0.0.1",9559)
print str(sys.argv[1]) + " " + str(memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Value")) + " " + str(memory.getData("Device/SubDeviceList/Battery/Charge/Sensor/Status")) + " " + str(len(os.listdir("/home/nao/logs")))