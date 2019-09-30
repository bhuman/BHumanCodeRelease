/*
 * @file GyroOffsetProvider.cpp
 * @author Philip Reichenberg
 */

#include "GyroOffsetProvider.h"

MAKE_MODULE(GyroOffsetProvider, infrastructure)

GyroOffsetProvider::GyroOffsetProvider()
{
  state = waiting;
  gyroValuesX.clear();
  gyroValuesY.clear();
  gyroValuesZ.clear();
  gyroChecks = { 0, 0, 0 };
  timestamps = { 0, 0, 0 };
  samplingStart = 0;
  samplingCounter = 0;
  wasPlayingOnce = false;
  lastGyros = Vector3a::Zero();
  lastGyroChange = 0;
  gyroStuckTimestamp = 0;
}

void GyroOffsetProvider::update(GyroOffset& gyroOffset)
{
  if(SystemCall::getMode() != SystemCall::physicalRobot)
    return;
  checkGyroDelay(gyroOffset);
  //Make sure the robot is not making sound when he is not in Initial.
  //I have no idea if this is enough in a case, where the game is already going on but the robot was rebooted.
  if(theGameInfo.state != STATE_INITIAL && wasPlayingOnce)
    return;
  if(theRobotInfo.penalty == 0 && (theGameInfo.state == STATE_PLAYING || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_READY))
  {
    wasPlayingOnce = true;
    return;
  }
  switch(state)
  {
    //waiting until the first time standHigh
    case waiting:
    {
      if(theGroundContactState.contact
         && theMotionRequest.motion == MotionRequest::specialAction
         && (theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::standHigh
             || theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead))
      {
        //is the robot long enough in standhigh?
        if(theFrameInfo.getTimeSince(samplingStart) > waitTime)
        {
          state = sampling;
          samplingCounter = 0;
        }
      }
      else
        samplingStart = theFrameInfo.time;
      break;
    }
    case sampling:
    {
      //if the motion request changed, we assume the robot starts playing now.
      if(theMotionRequest.motion != MotionRequest::specialAction
         || (theMotionRequest.specialActionRequest.specialAction != SpecialActionRequest::standHigh
             && theMotionRequest.specialActionRequest.specialAction != SpecialActionRequest::playDead))
      {
        state = waiting;
        break;
      }
      //when robot is picked up again, wait until he has ground contact
      if(!theGroundContactState.contact)
      {
        state = waiting;
        samplingCounter = 0;
        break;
      }
      //Sampling
      gyroValuesX.push_front(theInertialData.gyro.x());
      gyroValuesY.push_front(theInertialData.gyro.y());
      gyroValuesZ.push_front(theInertialData.gyro.z());
      samplingCounter += 1;
      //We did enough sampling
      if(samplingCounter >= static_cast<int>(gyroValuesX.capacity()))
        state = set;
      break;
    }
    case set:
    {
      std::vector<Vector3a> average;
      std::vector<Vector3a> deviation;
      RingBufferWithSum<Angle, 33> temp;
      int size = static_cast<int>(temp.capacity());
      for(int i = 0; i < 3; i++)
      {
        Vector3a averageInside = Vector3a();
        Vector3a deviationInside = Vector3a();
        //calc for x gyro
        temp.clear();
        for(int j = i * size; j < (i + 1) * size; j++)
          temp.push_front(gyroValuesX[j]);
        averageInside.x() = temp.average();
        deviationInside.x() = calcDeviation(temp);
        //calc for y gyro
        temp.clear();
        for(int j = i * size; j < (i + 1) * size; j++)
          temp.push_front(gyroValuesY[j]);
        averageInside.y() = temp.average();
        deviationInside.y() = calcDeviation(temp);
        //calc for z gyro
        temp.clear();
        for(int j = i * size; j < (i + 1) * size; j++)
          temp.push_front(gyroValuesZ[j]);
        averageInside.z() = temp.average();
        deviationInside.z() = calcDeviation(temp);
        average.push_back(averageInside);
        deviation.push_back(deviationInside);
      }
      //check if the windows of the last 200ms and the last 400-600ms the gyro deviation was small enough
      if(deviation[0].x() < thresholdGyroDeviation && deviation[0].y() < thresholdGyroDeviation && deviation[0].z() < thresholdGyroDeviation
         && deviation[2].x() < thresholdGyroDeviation && deviation[2].y() < thresholdGyroDeviation && deviation[2].z() < thresholdGyroDeviation)
      {
        gyroOffset.offset.x() = std::fabs(average[1].x()) > thresholdZero ? average[1].x() : Angle();
        gyroOffset.offset.y() = std::fabs(average[1].y()) > thresholdZero ? average[1].y() : Angle();
        gyroOffset.offset.z() = std::fabs(average[1].z()) > thresholdZero ? average[1].z() : Angle();
        if(std::fabs(average[1].x()) > thresholdZero
           || std::fabs(average[1].y()) > thresholdZero
           || std::fabs(average[1].z()) > thresholdZero)
        {
          SystemCall::playSound("sirene.wav");
          SystemCall::say((std::string("Gyro has Offset ") + TypeRegistry::getEnumName(Global::getSettings().teamColor) + " " + std::to_string(theRobotInfo.number)).c_str());
          ANNOTATION("GyroOffsetProvider", "Added Offset " << gyroOffset.offset);
          OUTPUT_TEXT("GyroOffsetProvider - Added an Offset for the Gyros.");
        }
        state = off;
      }
      else
      {
        samplingCounter -= 200;
        state = sampling;
      }
      break;
    }
    case off:
      break;
  }
}

Angle GyroOffsetProvider::calcDeviation(RingBufferWithSum<Angle, 33>& buffer)
{
  Angle deviation = Angle();
  deviation += std::fabs(buffer.maximum() - buffer.minimum());
  return deviation;
}

void GyroOffsetProvider::checkGyroDelay(GyroOffset& gyroOffset)
{
  if(lastGyroChange == 0 || lastGyros != theInertialData.gyro)
  {
    gyroOffset.gyroIsStuck = false;
    lastGyros = theInertialData.gyro,
    lastGyroChange = theFrameInfo.time;
  }
  else if(theFrameInfo.getTimeSince(lastGyroChange) > maxGyroDelay && lastGyroChange > startTimestamp)
  {
    gyroOffset.gyroIsStuck = true;
    if(theFrameInfo.getTimeSince(gyroStuckTimestamp) > waitTime)
    {
      gyroStuckTimestamp = theFrameInfo.time;
      ANNOTATION("GyroOffsetProvider", "No Gyro Update for " << theFrameInfo.getTimeSince(lastGyroChange) << "ms");
      SystemCall::playSound("sirene.wav");
      SystemCall::say((std::string("Gyro is wrong ") + TypeRegistry::getEnumName(Global::getSettings().teamColor) + " " + std::to_string(theRobotInfo.number)).c_str());
    }
  }
}
