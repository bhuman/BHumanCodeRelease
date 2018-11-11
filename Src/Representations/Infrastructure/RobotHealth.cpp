/**
 * @file RobotHealth.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author <a href="mailto:fthielke@uni-bremen.de">Felix Thielke</a>
 */

#include "RobotHealth.h"
#include "Tools/Module/Blackboard.h"

// otherwise RobotHealthMessageID::getName() throws an unused function warning
struct WarningSuppressor
{
  ENUM(RobotHealthMessageId,
  {,
    idmotionFrameRate,
    idavgMotionTime,
    idmaxMotionTime,
    idminMotionTime,
    idcognitionFrameRate,
    idbatteryLevel,
    idtotalCurrent,
    idmaxJointTemperatureStatus,
    idjointWithMaxTemperature,
    idcpuTemperature,
    idload,
    idmemoryUsage,
    idwlan,
    idrobotName,
    idconfiguration,
    idlocation,
    idscenario,
  });
};

#define SEND_CASE(value) case WarningSuppressor::id##value: m.theBHumanArbitraryMessage.queue.out.bin << value; break
#define SEND_CASE_ARRAY(value) case WarningSuppressor::id##value: for(size_t _i=0;_i<value.size();++_i) m.theBHumanArbitraryMessage.queue.out.bin << value[_i]; break
void RobotHealth::operator>>(BHumanMessage& m) const
{
  WarningSuppressor::RobotHealthMessageId idToSend = WarningSuppressor::idbatteryLevel;
  if(Blackboard::getInstance().exists("BHumanMessageOutputGenerator"))
    idToSend = WarningSuppressor::RobotHealthMessageId(static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sentMessages % WarningSuppressor::numOfRobotHealthMessageIds);

  m.theBHumanArbitraryMessage.queue.out.bin << idToSend;
  switch(idToSend)
  {
      SEND_CASE(motionFrameRate);
      SEND_CASE(avgMotionTime);
      SEND_CASE(maxMotionTime);
      SEND_CASE(minMotionTime);
      SEND_CASE(cognitionFrameRate);
      SEND_CASE(batteryLevel);
      SEND_CASE(totalCurrent);
      SEND_CASE(maxJointTemperatureStatus);
      SEND_CASE(jointWithMaxTemperature);
      SEND_CASE(cpuTemperature);
      SEND_CASE_ARRAY(load);
      SEND_CASE(memoryUsage);
      SEND_CASE(wlan);
      SEND_CASE(robotName);
      SEND_CASE(configuration);
      SEND_CASE(location);
      SEND_CASE(scenario);
    default:
      FAIL("" << idToSend << " has no matching value");
  }

  m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
}

#define RECEIVE_CASE(value) case WarningSuppressor::id##value: m.bin >> value; break
#define RECEIVE_CASE_ARRAY(value) case WarningSuppressor::id##value: for(size_t _i=0;_i<value.size();++_i) m.bin >> value[_i]; break
bool RobotHealth::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());

  WarningSuppressor::RobotHealthMessageId receivedRHID;
  m.bin >> receivedRHID;
  switch(receivedRHID)
  {
      RECEIVE_CASE(motionFrameRate);
      RECEIVE_CASE(avgMotionTime);
      RECEIVE_CASE(maxMotionTime);
      RECEIVE_CASE(minMotionTime);
      RECEIVE_CASE(cognitionFrameRate);
      RECEIVE_CASE(batteryLevel);
      RECEIVE_CASE(totalCurrent);
      RECEIVE_CASE(maxJointTemperatureStatus);
      RECEIVE_CASE(jointWithMaxTemperature);
      RECEIVE_CASE(cpuTemperature);
      RECEIVE_CASE_ARRAY(load);
      RECEIVE_CASE(memoryUsage);
      RECEIVE_CASE(wlan);
      RECEIVE_CASE(robotName);
      RECEIVE_CASE(configuration);
      RECEIVE_CASE(location);
      RECEIVE_CASE(scenario);
    default:
      FAIL("" << receivedRHID << " has no matching value");
      break;
  }

  return true;
}
