#!/usr/bin/python
import naoqi
import sys
from naoqi import ALProxy

# call method
try:
  alSystem = ALProxy("ALSystem", "127.0.0.1", 9559)
  robotName = sys.argv[1]
  alSystem.setRobotName(robotName)
  print "Changed robot name to " + robotName
  exit(0)

except RuntimeError,e:
  print "Failed to change robot name:"
  print str(e)
  exit(1)
