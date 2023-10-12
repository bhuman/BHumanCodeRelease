/**
 * @file RefereeGestureDetectionNN.cpp
 *
 * This file implements a module that <#...#>
 *
 * @author Ayleen Lührsen
 */

#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "RefereeGestureDetectionNN.h"
#include <CompiledNN2ONNX/Model.h>

MAKE_MODULE(RefereeGestureDetectionNN);

RefereeGestureDetectionNN::RefereeGestureDetectionNN()
  : detector(&Global::getAsmjitRuntime())
{
#ifdef TARGET_ROBOT
  compileNetwork();
#endif
}

void RefereeGestureDetectionNN::update(RefereePercept& theRefereePercept)
{
  if(!theOptionalCameraImage.image.has_value())
  {
    gestureMemory.clear();
    return;
  }
  if(!isCompiled)
  {
    compileNetwork();
  }

  // Copied from Thomas' Module
  // Only update history in SimRobot at a similar rate as on the real robot.
  if(SystemCall::getMode() == SystemCall::physicalRobot
     || Time::getTimeSince(lastBufferUpdate) > 285
     || Time::getTimeSince(lastBufferUpdate) < 0)
  {
    lastBufferUpdate = Time::getCurrentSystemTime();

    theRefereePercept.gesture = RefereePercept::Gesture::none;
    // Our poseCandidate is none to begin with
    RefereePercept::Gesture gestureCandidate = RefereePercept::Gesture::none;
    if(gestureMemory.full())
    {
      gestureAppearances.fill(0);
      for(std::size_t i = 0; i < gestureMemory.capacity(); i++)
      {
        gestureAppearances[gestureMemory[i]]++;
      }

      // based on the number of appearances, find the pose that was estimated the most times over the last frames
      gestureCandidate = static_cast<RefereePercept::Gesture>(std::max_element(gestureAppearances.begin(), gestureAppearances.end()) - gestureAppearances.begin());
      if(gestureAppearances[gestureCandidate] > static_cast<int>(gestureMemory.capacity() / bufferPortion))
      {
        theRefereePercept.gesture = gestureCandidate;
      }
    }

    const int directionFactor = theGameState.leftHandTeam ? 1 : -1;
    const Vector2f absRefereePosition = Vector2f(theFieldDimensions.xPosHalfWayLine, -(theFieldDimensions.yPosRightSideline * directionFactor));

    // Add input to model.
    float* input = detector.input(0).data();

    // add distance to referee in meters and scaled down to the input
    *input++ = ((theRobotPose.translation - absRefereePosition).norm() * 0.1f) / 1000.f;
    *input++ = -Angle(Angle::normalize((theRobotPose.translation - absRefereePosition).angle() + 90_deg)).toDegrees() * 0.1f;;

    const CameraImage& theCameraImage = theOptionalCameraImage.image.value();

    //  Average position of keypoint in image on both axes. If keypoints deviate to much or are to clustered, the gesture will be invalid.
    float avrgX = 0.f;
    float avrgY = 0.f;
    unsigned validPoints = 0;
    // Add keypoints to input
    for(int i = Keypoints::leftShoulder; i <= Keypoints::rightHip; i++)
    {
      const Keypoints::Point& point = theKeypoints.points[i];
      // if you reach one of the wanted joints, save them into the percept
      if(point.valid)
      {
        const float x = ((point.position.x() - theCameraImage.width) / theKeypoints.patchBoundary.x.getSize()) + 0.5f;
        const float y = ((point.position.y() - (theCameraImage.height / 2)) / theKeypoints.patchBoundary.y.getSize()) + 0.5f;
        *input++ = y;
        *input++ = x;

        // Add to sum to calculate average later
        validPoints++;
        avrgX += x;
        avrgY += y;
      }
      else
      {
        *input++ = 0.f;
        *input++ = 0.f;
      }
    }
    ASSERT(detector.input(0).data() + 18 == input);

    avrgX /= validPoints;
    avrgY /= validPoints;

    // MoveNet probably got the wrong person, move on. hahah.
    if(!validXAvrg.isInside(avrgX) || !validYAvrg.isInside(avrgY))
    {
      theRefereePercept.gesture = RefereePercept::Gesture::none;
      return;
    }

    detector.apply();

    const float* output = detector.output(0).data();
    float maxProb = 0.f;
    FOREACH_ENUM(RefereePercept::Gesture, gesture)
    {
      if(gesture == RefereePercept::Gesture::none ||
         gesture == RefereePercept::Gesture::substitutionRed)
      {
        continue;
      }
      detectorOutput[gesture] = *output++;
      if(detectorOutput[gesture] >= maxProb)
      {
        maxProb = detectorOutput[gesture];
        gestureCandidate = gesture;
      }
      //    We have two full time poses in training but cant use it as enum, so ,ake a special case here
      if(gesture == RefereePercept::fullTime)
      {
        detectorOutput[gesture] = *output++;
        if(detectorOutput[gesture] >= maxProb)
        {
          maxProb = detectorOutput[gesture];
          gestureCandidate = gesture;
        }
      }
    }
    gestureMemory.push_front(gestureCandidate);
    //OUTPUT_TEXT("OUTPUT START");
    //OUTPUT_TEXT(output[-13]<<","<<output[-12]<<","<<output[-11]<<","<<output[-10]<<","<<output[-9]<<","<<output[-8]<<","<<output[-7]<<","<<output[-6]<<","<<output[-5]<<","<<output[-4]<<","<<output[-3]<<","<<output[-2]<<","<<output[-1]);
  }
}

void RefereeGestureDetectionNN::compileNetwork()
{
  NeuralNetworkONNX::CompilationSettings settings;
#if defined MACOS && defined __arm64__
  settings.useCoreML = true;
#endif
  isCompiled = true;
  detector.compile(NeuralNetworkONNX::Model(std::string(File::getBHDir()) + "/Config/NeuralNets/RefereeGestureDetection/230630021423.h5"),
                   settings);

  ASSERT(detector.input(0).rank() == 1);
  ASSERT(detector.numOfInputs() == 1);
  ASSERT(detector.input(0).dims(0) == 18);
  ASSERT(detector.output(0).rank() == 1);
  ASSERT(detector.numOfOutputs() == 1);
  ASSERT(detector.output(0).dims(0) == 13);
}
