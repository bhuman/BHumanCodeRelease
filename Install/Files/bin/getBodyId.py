#!/usr/bin/python
import naoqi
from naoqi import ALProxy

# call method
try:
  dcm = ALProxy("ALMemory", "127.0.0.1", 9559)
  id = dcm.getData("Device/DeviceList/ChestBoard/BodyId")
  print id
  exit(0)

except RuntimeError,e:
  print "Fail"
  exit(1)
