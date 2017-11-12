#include "JointAnglesProvider.h"
#include <sstream>
#include <iomanip>

MAKE_MODULE(JointAnglesProvider, motionInfrastructure)

void JointAnglesProvider::update(JointAngles& jointAngles)
{
  jointAngles = theJointSensorData;
  if(!theRobotInfo.hasFeature(RobotInfo::wristYaws))
  {
    jointAngles.angles[Joints::lWristYaw] = -90_deg;
    jointAngles.angles[Joints::rWristYaw] = 90_deg;
  }
  DEBUG_RESPONSE_ONCE("module:JointAnglesProvider")
  {
    std::string str = "HY____ HP____ LSP___ LSR___ LEY___ LER___ LWY___ LH____ RSP___ RSR___ REY___ RER___ RWY___ RH____ LHYP__ LHR___ LHP___ LKP___ LAP___ LAR___ RHYP__ RHR___ RHP___ RKP___ RAP___ RAR___ Int  Dur\n";
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      float ang = jointAngles.angles[i].toDegrees();
      std::ostringstream a;
      a << std::setprecision(1) << std::fixed << ang;
      std::string angle(a.str());
      int nums(6 - (int)angle.size());
      while(nums >= 0)
      {
        angle += " ";
        --nums;
      }
      str += angle;
    }
    OUTPUT(idText, text, str);
  }
}