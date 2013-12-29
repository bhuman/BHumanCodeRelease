/**
* @file FrictionLearner.h
*
* This file implements a module which can learn the ball friction.
*
* @author Martin Böschen
*/

#include "Tools/Streams/InStreams.h"
#include "FrictionLearner.h"
#include "cmath"

void FrictionLearner::updateAndOutput(float vec, float acc)
{
  if(std::abs(vec)>=15)
  {
    estimate_nom += vec * acc;
    estimate_denom += vec * vec;
    numSamples++;
    OUTPUT(idText, text, numSamples << ", Velocity: " << vec << ", Estimate: " << estimate_nom/estimate_denom*(1/theCognitionFrameInfo.cycleTime) );
  }
}

void FrictionLearner::handleBallPercept(CameraInfo::Camera c, std::vector<Vector2<>>& ballPercepts)
{
  if(theCameraInfo.camera == c)
  {
    if(theBallPercept.ballWasSeen )
    {
      ballPercepts.push_back(theBallPercept.relativePositionOnField);
      if(ballPercepts.size() >= 9)
      {
        float vx,vy,ax,ay;
        vx = vy = ax = ay = 0.0f;
        int offset = ballPercepts.size() - 9;
        for(int i = 0; i < 9; i++)
        {
          vy += ballPercepts[i+offset].y * firstDirCoeffs[i];
          ay += ballPercepts[i+offset].y * secondDirCoeffs[i];
          vx += ballPercepts[i+offset].x * firstDirCoeffs[i];
          ax += ballPercepts[i+offset].x * secondDirCoeffs[i];
        }
        vx /= 1188.0f; vy /= 1188.0f;
        ax /= 56628.0f; ay /= 56628.0f;
        updateAndOutput(vx, ax);
        updateAndOutput(vy, ay);
      }
    }
    else ballPercepts.clear();
  }
}

void FrictionLearner::update(FieldDimensions& fieldDimensions)
{
  if(collectData)
  {	
    // data of ball percept is simply saved into a list, which can be written into a file for later offline analysing
    if(theBallPercept.ballWasSeen)
    {
      ballPercept s;
      s.time = theFrameInfo.time;
      s.camera = theCameraInfo.camera;
      s.x = theBallPercept.relativePositionOnField.x;
      s.y = theBallPercept.relativePositionOnField.y;
      allBallPercepts.push_back(s);
    }

    handleBallPercept(CameraInfo::upper, upperCameraPercepts);
    handleBallPercept(CameraInfo::lower, lowerCameraPercepts);
  }

  DEBUG_RESPONSE_ONCE("module:FrictionLearner:writeDataToFile",
    writeToFile();
  );

  DEBUG_RESPONSE_ONCE("module:FrictionLearner:startCollectData",
    collectData = true;
  );

  DEBUG_RESPONSE_ONCE("module:FrictionLearner:stopCollectData",
    collectData = false;
  );

  DEBUG_RESPONSE_ONCE("module:FrictionLearner:resetEstimator",
    upperCameraPercepts.clear();
    lowerCameraPercepts.clear();
    collectData = false;
    estimate_nom = 0.0f;
    estimate_denom = 0.0f;
    numSamples = 0;
  );
}

void FrictionLearner::writeToFile()
{
  OutTextFile file("trajectories.csv", false);
  file << "camera,time,xpos,ypos" << endl;
  for(auto iter = allBallPercepts.begin(); iter != allBallPercepts.end(); iter++)
    file << iter->camera << "," << iter->time << "," << iter->x << "," << iter->y << endl;
}

MAKE_MODULE(FrictionLearner, Cognition Infrastructure)