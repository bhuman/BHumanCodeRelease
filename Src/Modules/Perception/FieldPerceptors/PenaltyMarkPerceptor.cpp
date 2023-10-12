/**
 * @file PenaltyMarkPerceptor.cpp
 *
 * This file implements a module that detects penalty marks in images with a neural network.
 *
 * @author Simon Werner
 * @author Finn Ewers
 * @author Thomas Röfer
 */

#include "PenaltyMarkPerceptor.h"
#include "Debugging/Stopwatch.h"
#include "Platform/File.h"
#include "Tools/Math/InImageSizeCalculations.h"

MAKE_MODULE(PenaltyMarkPerceptor);

PenaltyMarkPerceptor::PenaltyMarkPerceptor() :
  network(&Global::getAsmjitRuntime())
{
  model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir())
    + "/Config/NeuralNets/PenaltyMarkPerceptor/penalty_model.h5");
  network.compile(*model);
}

void PenaltyMarkPerceptor::update(PenaltyMarkPercept& thePenaltyMarkPercept)
{
  thePenaltyMarkPercept.wasSeen = false;

  // Special handling for penalty shootout: the penalty mark is where the ball is.
  if(theGameState.isPenaltyShootout())
  {
    if(theBallPercept.status == BallPercept::Status::seen)
    {
      thePenaltyMarkPercept.positionInImage = theBallPercept.positionInImage.cast<int>();

      // Todo use covariance of ball percept
      if(theMeasurementCovariance.transformWithCov(theBallPercept.positionInImage, theBallSpecification.radius, thePenaltyMarkPercept.positionOnField,
                                                   thePenaltyMarkPercept.covarianceOnField))
        thePenaltyMarkPercept.wasSeen = true;
      return;
    }
  }

  if(!network.valid() || thePenaltyMarkRegions.regions.empty())
    return;

  for(const Boundaryi& region : thePenaltyMarkRegions.regions)
  {
    STOPWATCH("module:PenaltyMarkPerceptor:network")
    {
      //extract patch
      float width, height;
      const Vector2f center(region.x.getCenter(), region.y.getCenter());
      if(!IISC::calculateImagePenaltyMeasurementsByCenter(center, width, height, theCameraInfo, theCameraMatrix, theFieldDimensions))
        continue;

      const int inputSize = static_cast<int>(std::max(width, height) * 1.7f);
      PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(32, 32), theECImage.grayscaled, network.input(0).data());

      network.apply();

      //do something with prediction
      const float first = network.output(0)[0];
      const float second = network.output(0)[1];
      const float confidence = second - first;

      if(first < second && confidence > threshold
         && theMeasurementCovariance.transformWithCov(center, 0.f, thePenaltyMarkPercept.positionOnField, thePenaltyMarkPercept.covarianceOnField))
      {
        thePenaltyMarkPercept.positionInImage = center.cast<int>(); // todo: estimate actual center
        thePenaltyMarkPercept.wasSeen = true;
        break;
      }
    }
  }
}
