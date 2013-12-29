import os
import sys
import time

path = `os.environ.get("AL_DIR")`
home = `os.environ.get("HOME")`

# import naoqi lib
import naoqi
from naoqi import ALBroker
from naoqi import ALModule
from naoqi import ALProxy

# call method
try:
  dcm = ALProxy("DCM", "127.0.0.1", 9559)
  robotName = sys.argv[1]
  dcm.preferences("Add", "Chest","Device/DeviceList/ChestBoard/BodyNickName", robotName)
  time.sleep(1)
  dcm.preferences("Save", "Chest", "", 0.0) # to save new calibration
  print "Changed body nick name to " + robotName
  exit(0)

except RuntimeError,e:
  print "Failed to change body nick name"
  exit(1)
