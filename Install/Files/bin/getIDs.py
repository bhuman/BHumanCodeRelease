import naoqi
from naoqi import ALProxy

# call method
try:
  dcm = ALProxy("ALMemory", "127.0.0.1", 9559)
  id1 = dcm.getData("RobotConfig/Body/BaseVersion")
  id2 = dcm.getData("RobotConfig/Body/Version")
  id3 = dcm.getData("RobotConfig/Head/Version")
  id4 = dcm.getData("RobotConfig/Head/Type")
  print ("Body BaseVersion: " + id1 + " " + "Body Version: " + id2 + " " +\
          "Head Version: " + id3 + " " + "Head Type: " + id4)
  exit(0)

except RuntimeError,e:
  print "Fail"
  exit(1)
