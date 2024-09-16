/**
 * @file WalkLearnerProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "WalkLearnerProvider.h"
#include <algorithm>

MAKE_MODULE(WalkLearnerProvider);

WalkLearnerProvider::WalkLearnerProvider()
{
  gyroForwardBalanceFactor = -1.f;
  gyroBackwardBalanceFactor = -1.f;
  oldForwardGyro = 0.f;
  oldAverageMaxGyro = 0.f;
  oldAverageMaxGyroPhase1 = 0.f;
  oldBackwardGyro = 0.f;
  oldAverageMinGyro = 9000.f;
  oldAverageMinGyroPhase1 = 9000.f;
  phaseLearn = 0;
  learnIteration = 0;
  isForwardPhase = false;
  wasPositive = false;
}

void WalkLearnerProvider::update(WalkLearner& walkLearner)
{
  // set functions and copy the balance factors from the walkGenerator
  if(gyroForwardBalanceFactor >= 0.f)
    walkLearner.useWalkLearner = true;
  walkLearner.setBaseWalkParams = [this](float gyroForward, float gyroBackward, float speedTransX)
  {
    setBaseWalkParams(gyroForward, gyroBackward, speedTransX);
  };

  walkLearner.updateStepDuration = [this, &walkLearner](const Pose2f& lastStep, const float lastDuration, const float refStepDuration)
  {
    learnStepHeightDuration(lastStep, lastDuration, refStepDuration, walkLearner.stepHeightDurationOffset);
  };

  walkLearner.newGyroBackwardBalance = gyroBackwardBalanceFactor;
  walkLearner.newGyroForwardBalance = gyroForwardBalanceFactor;
  learnGyroBalanceFactor(walkLearner);
}

void WalkLearnerProvider::setBaseWalkParams(float gyroForward, float gyroBackward, float speedTransX)
{
  if(gyroForwardBalanceFactor < 0.f)
  {
    gyroForwardBalanceFactor = gyroForward;
    gyroBackwardBalanceFactor = gyroBackward;
  }
  speed = speedTransX;
}

void WalkLearnerProvider::learnGyroBalanceFactor(WalkLearner&)
{
  if(gyroForwardBalanceFactor < 0.f)
    return;
  //numOfGyro was changed. Init the variables.
  if(theWalkModifier.numOfGyroPeaks != static_cast<int>(gyroForwardMax.size()) || theWalkModifier.numOfGyroPeaks != static_cast<int>(gyroBackwardMin.size()))
  {
    gyroForwardMax = std::vector<float>(theWalkModifier.numOfGyroPeaks, 0.f);
    gyroBackwardMin = std::vector<float>(theWalkModifier.numOfGyroPeaks, 0.f);
    learnIteration = 0;
    isForwardPhase = true;
    wasPositive = true;
  }

  //If numOfGyro is > 0 and the robot walks over 100mm/s, then go into the learning part
  if(theWalkModifier.numOfGyroPeaks > 0 && speed > 25.f)
  {
    //We are sampling over the positive gyro peaks
    if(isForwardPhase && wasPositive)
    {
      if(theInertialData.gyro.y() > gyroForwardMax[learnIteration])
        gyroForwardMax[learnIteration] = theInertialData.gyro.y();
      else if(theInertialData.gyro.y() < 0.f)
        isForwardPhase = !isForwardPhase;
    }
    //We are sampling over the negative gyro peaks
    else if(!isForwardPhase && !wasPositive)
    {
      if(theInertialData.gyro.y() < gyroBackwardMin[learnIteration])
        gyroBackwardMin[learnIteration] = theInertialData.gyro.y();
      else if(theInertialData.gyro.y() > 0.f)
        isForwardPhase = !isForwardPhase;
      if(learnIteration + 1 <= theWalkModifier.numOfGyroPeaks && isForwardPhase)
        learnIteration += 1;
    }
    //This is needed to be sure, that we collect a negative gyro peak after a positive gyro peak was collected and the
    //sign has changed. Same vice versa.
    else
      wasPositive = (isForwardPhase && theInertialData.gyro.y() > 0.f) || (!isForwardPhase && theInertialData.gyro.y() < 0.f) ? !wasPositive : wasPositive;
    //We collected enough gyro peaks. Rate them and adjust gyroBalanceFactor
    if(learnIteration >= theWalkModifier.numOfGyroPeaks)
    {
      learnIteration = 0;
      float number = 0;
      if(phaseLearn > 0 && phaseLearn < 3)
        number = 1.1f;
      else if(phaseLearn > 4 || phaseLearn == 0)
      {
        phaseLearn = 0;
        number = Random::uniform(0.f, 2.f); // there is definitely a better solution to do random deciding
        // which gyro shall get changed next. But I, Philip, am too lazy for it
        // right now.
      }
      float average = 0.f;
      float averageDiff = 0.f;
      //calculate average value and average deviation (standard deviation could be tested?)
      //averageDiff will get used to change the gyros for balancing
      //Change gyroForwardBalanceFactor
      if(number > 1.f)
      {
        for(float f : gyroForwardMax)
          average += f;
        if(average != 0.f)
          average = average / theWalkModifier.numOfGyroPeaks;

        for(float f : gyroForwardMax)
          averageDiff += (f - average) * (f - average);
        if(averageDiff != 0.f)
          averageDiff = averageDiff / theWalkModifier.numOfGyroPeaks;

        //Rate gyro peaks without change
        if(phaseLearn == 0)
        {
          phaseLearn += 1;
          oldForwardGyro = gyroForwardBalanceFactor;
          gyroForwardBalanceFactor += averageDiff * theWalkModifier.balanceChangeFactor;
          gyroForwardBalanceFactor = std::max(gyroForwardBalanceFactor, 0.f);
          oldAverageMaxGyro = average * averageDiff;
        }
        //Rate gyro peaks with first change
        else if(phaseLearn == 1)
        {
          phaseLearn += 1;
          gyroForwardBalanceFactor += 2.f * (oldForwardGyro - gyroForwardBalanceFactor);
          gyroForwardBalanceFactor = std::max(gyroForwardBalanceFactor, 0.f);
          oldAverageMaxGyroPhase1 = average * averageDiff;
        }
        //Rate gyro peaks with second change and take the best version of all 3.
        else
        {
          phaseLearn = 10;
          if(oldAverageMaxGyro < oldAverageMaxGyroPhase1 && oldAverageMaxGyro <= average * averageDiff)
            gyroForwardBalanceFactor = oldForwardGyro;
          else if(oldAverageMaxGyro > oldAverageMaxGyroPhase1 && oldAverageMaxGyro > average * averageDiff)
            gyroForwardBalanceFactor += 2.f * (oldForwardGyro - gyroForwardBalanceFactor);
        }
      }
      //Change gyroBackwardBalanceFactor
      else
      {
        for(float f : gyroBackwardMin)
          average += f;
        if(average != 0.f)
          average = average / theWalkModifier.numOfGyroPeaks;

        for(float f : gyroBackwardMin)
          averageDiff += (f - average) * (f - average);
        if(averageDiff != 0.f)
          averageDiff = averageDiff / theWalkModifier.numOfGyroPeaks;

        //Rate gyro peaks without change
        if(phaseLearn == 0)
        {
          phaseLearn = 3;
          oldBackwardGyro = gyroBackwardBalanceFactor;
          gyroBackwardBalanceFactor += averageDiff * theWalkModifier.balanceChangeFactor;
          gyroBackwardBalanceFactor = std::max(gyroBackwardBalanceFactor, 0.f);
          oldAverageMinGyro = average * averageDiff;
        }
        //Rate gyro peaks with first change
        else if(phaseLearn == 3)
        {
          phaseLearn += 1;
          gyroBackwardBalanceFactor += 2.f * (oldBackwardGyro - gyroBackwardBalanceFactor);
          gyroBackwardBalanceFactor = std::max(gyroBackwardBalanceFactor, 0.f);
          oldAverageMinGyroPhase1 = average * averageDiff;
        }
        //Rate gyro peaks with second change and take the best version of all 3.
        else
        {
          phaseLearn = 10;
          if(oldAverageMinGyro < oldAverageMinGyroPhase1 && oldAverageMinGyro <= average * averageDiff)
            gyroBackwardBalanceFactor = oldBackwardGyro;
          else if(oldAverageMinGyro > oldAverageMinGyroPhase1 && oldAverageMinGyro > average * averageDiff)
            gyroBackwardBalanceFactor += 2.f * (oldBackwardGyro - gyroBackwardBalanceFactor);
        }
      }
      //reset list with the saved gyro peaks.
      gyroForwardMax = std::vector<float>(theWalkModifier.numOfGyroPeaks, 0.f);
      gyroBackwardMin = std::vector<float>(theWalkModifier.numOfGyroPeaks, 0.f);
    }
  }
}

void WalkLearnerProvider::learnStepHeightDuration(const Pose2f& lastStep, const float lastDuration, const float refStepDuration, float& currentAdjustment)
{
  if(std::abs(lastStep.rotation) > maxTurnStep || std::abs(lastStep.translation.y()) > maxSideStep || std::abs(lastStep.translation.x()) < minForwardStep)
    return;

  stepDurationBuffer.push_front(std::min(lastDuration, refStepDuration * clipStepDurationRatio));
  if(stepDurationBuffer.full() && theJointPlay.isCalibrated)
  {
    if(adjustmentCounter < maxStepsForJointPlayInitializing)
      currentAdjustment = (1.f - theJointPlay.qualityOfRobotHardware) * jointPlayInitializing;
    else
    {
      float adjustment = mapToRange(Rangef::ZeroOneRange().limit(adjustmentCounter / adjustmentSteps), 0.f, 1.f, adjustmentRange.min, adjustmentRange.max);
      const float useRefStepDuration = refStepDuration * mapToRange((1.f - theJointPlay.qualityOfRobotHardware), 0.f, 1.f, bestStepDuration.min, bestStepDuration.max);
      if(stepDurationBuffer.average() > useRefStepDuration)
        adjustment *= -1.f;
      currentAdjustment += adjustment;
    }

    currentAdjustment = std::max(0.f, currentAdjustment);

    adjustmentCounter++;
    stepDurationBuffer.clear();
  }
}
